import sys
import time

from enum import IntEnum, auto
from my_interfaces.srv import Detect
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8

dest_ = [[-1.2, 0.8], [1.0, 2.0], [1.0, 2.0]]

class State(IntEnum):
    INIT = auto()
    INPUT = auto()
    WAIT = auto()
    SEND = auto()
    ERROR = auto()

class MinimalClientAsync(Node):

    def __init__(self):
        self.state = State.INIT
        super().__init__('import_system')
        self.cli = self.create_client(Detect, 'detection')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Detect.Request()
        
        self.subscription = self.create_subscription(
            Int8,
            'result',
            self.result_callback,
            10)

        self.state = State.INPUT
        self.subscription

    #/resultに結果が返って来たときの処理
    def result_callback(self, msg):
        if msg.data == 1:
            print('bringing success!!')
        elif msg.data == 11:
            print('object detection false...')
        elif msg.data == 21:
            print('object carrying false...')
        self.state = State.INPUT

    #捜し物検知システムにリクエスとを送る
    def send_request(self, name, dest):
        self.req.name = name
        for i in range(len(dest_)):
            if(i == int(dest)):
                self.req.dest = dest_[i]
                break
            if(i == len(dest_)-1):
                print("select correct destination!")
                self.state = State.INPUT
                return
        
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    
    while rclpy.ok():
        #入力状態
        if minimal_client.state == State.INPUT:
            Name = input("探し物名:")
            Dest = input("届け先:")
            minimal_client.state = State.SEND
        #送信状態
        elif minimal_client.state == State.SEND:
            response = minimal_client.send_request(Name, Dest)
            if response.res == 1:
                minimal_client.state = State.WAIT
            else:
                print('object_detect_system busy!')
                minimal_client.state = State.INPUT
        #運搬結果受信待機状態
        elif minimal_client.state == State.WAIT:
            rclpy.spin_once(minimal_client)          

        time.sleep(0.1)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
