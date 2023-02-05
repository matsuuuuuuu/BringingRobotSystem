import time
import sys
from enum import IntEnum, auto
from std_msgs.msg import Int8
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose

from my_interfaces.srv import Carry
from rasptank_msgs.srv import Grub, Release

import rclpy

from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

class State(IntEnum):
    INIT = auto()
    WAIT = auto()
    TOOBJ = auto()
    GRUB = auto()
    TOGOAL = auto()
    RELEASE = auto()
    ERROR = auto()


class BasicNavigator(Node):
    def __init__(self):
        self.state = State.INIT

        super().__init__(node_name='carry_system')
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None

        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = 0.0
        self.goal_pose.pose.position.y = 0.0
        self.goal_pose.pose.position.z = 0.0
        self.goal_pose.pose.orientation.x = 0.0
        self.goal_pose.pose.orientation.y = 0.0
        self.goal_pose.pose.orientation.z = 0.0
        self.goal_pose.pose.orientation.w = 1.0

        self.to = []
        self.dest = []
        self.hsv = []

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
   
        self.publisher_ = self.create_publisher(Int8, 'result', 10)
        self.msg = Int8()

        self.initial_pose_received = False
        self.cli_grub = self.create_client(Grub, 'grub')
        while not self.cli_grub.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_grub = Grub.Request()

        self.cli_release = self.create_client(Release, 'release')
        while not self.cli_release.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req_release = Release.Request()

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.srv = self.create_service(Carry, 'carrying', self.recieve_pos_callback)

        self.state = State.WAIT
    
    #運搬結果を/resultに送信する
    def publish_result(self):
        self.publisher_.publish(self.msg)

    #grub_objectに物体を下ろす依頼を行う
    def send_release_request(self):
        self.req_release = True
        future = self.cli_release.call_async(self.req_release)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.state = State.WAIT
        else:
            self.state = State.ERROR
        return

    #grub_objectに物体を掴む依頼を行う
    def send_grub_request(self):
        self.req_grub = True
        future = self.cli_grub.call_async(self.req_grub)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            self.state = State.TOGOAL
        else:
            self.state = State.ERROR
        return #self.future.result()
        
    #object_detect_systemから運搬依頼を受け取る
    def recieve_pos_callback(self, request, response):
        if(self.state == State.WAIT):
            response.res = True
        else:
            response.res = False
            return response
        self.to = request.pos
        self.dest = request.dest
        self.req_grub.hsv = request.hsv
        self.state = State.TOOBJ

        return response

    #引数の座標までナビゲーションを行うメソッド
    def goToPose(self, posx, posy):
        # Sends a `NavToPose` action request and waits for completion
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")
        
        self.goal_pose.pose.position.x = float(posx)
        self.goal_pose.pose.position.y = float(posy)

        #Navigation2にナビゲーションを依頼する
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_pose

        self.info('Navigating to goal: ' + str(self.goal_pose.pose.position.x) + ' ' +
                      str(self.goal_pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        #Navigation2が別の処理中の場合
        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(self.goal_pose.pose.position.x) + ' ' +
                           str(self.goal_pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    #ナビゲーションのキャンセルを行うメソッド
    def cancelNav(self):
        self.info('Canceling current goal.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    #ナビゲーションが完了した際に呼び出されるメソッド
    def isNavComplete(self):
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.info('Goal with failed with status code: {0}'.format(self.status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.info('Goal succeeded!')
        return True

    def getFeedback(self):
        return self.feedback

    def getResult(self):
        return self.status

    def waitUntilNav2Active(self):
        self._waitForNodeToActivate('amcl')
        self._waitForInitialPose()
        self._waitForNodeToActivate('bt_navigator')
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug('Waiting for ' + node_name + ' to become active..')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(node_service + ' service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while (state != 'active'):
            self.debug('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug('Result of get_state: %s' % state)
            time.sleep(2)
        return

    def _waitForInitialPose(self):
        while not self.initial_pose_received:
            self.info('Setting initial pose')
            self._setInitialPose()
            self.info('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1)
        return

    def _amclPoseCallback(self, msg):
        self.initial_pose_received = True
        return

    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

    

def main(argv=sys.argv[1:]):
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for navigation to fully activate
    #navigator.waitUntilNav2Active()

    
    i = 0

    while rclpy.ok():

        #運搬依頼待機状態
        if navigator.state == State.WAIT:
            rclpy.spin_once(navigator)

        #探し物の位置まで移動状態
        elif navigator.state == State.TOOBJ:
            #navigator.send_grub_request() #debug
            print('state : TOOBJ')
            navigator.goToPose(navigator.to[0], navigator.to[1])
            while not navigator.isNavComplete():

                # Do something with the feedback
                #i = i + 1
                feedback = navigator.getFeedback()

            navigator.state = State.GRUB
            result = navigator.getResult()
            if result != GoalStatus.STATUS_SUCCEEDED:
                print('Goal failed')
                navigator.state = State.ERROR
            
        #探し物の持ち上げ状態
        elif navigator.state == State.GRUB:
            print('state : GRUB')
            navigator.send_grub_request()

        #届け先の位置まで移動状態
        elif navigator.state == State.TOGOAL:
            print('state : TOGOAL')
            navigator.goToPose(navigator.dest[0], navigator.dest[1])
            while not navigator.isNavComplete():
                feedback = navigator.getFeedback()
            result = navigator.getResult()
            navigator.state = State.RELEASE
            #if result != GoalStatus.STATUS_SUCCEEDED:
            #    print('Goal failed')
            #    navigator.state = State.ERROR

        #探し物を下ろす状態
        elif navigator.state == State.RELEASE:
            print('state : RELEASE')
            navigator.send_release_request()
            navigator.msg.data = 1
            navigator.publish_result()
        #エラー状態
        elif navigator.state == State.ERROR:
            navigator.msg.data = 21
            navigator.publish_result()
            print('state : ERROR')
            navigator.state = State.WAIT

        time.sleep(0.1)

    exit(0)

