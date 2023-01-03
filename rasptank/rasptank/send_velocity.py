import math
import sys

import time

from geometry_msgs.msg import Twist


import rclpy
from rclpy.node import Node

from .control_modules.move import *

from rasptank_msgs.msg import Velocity


linear_min = 0.10 # #your robot min linear velocity [m/s]
angular_min = 0.7 # #your robot min angular velocity [rad/s]
radius_min = 0.6


#linear_interval = 0.015 #your robot linear velocity [m/s]
#angular_interval = 0.15  #your robot angular velocity [rad/s]
#radius_interval = 0.1

speed_set = 70

class SimPublisher(Node):

    def __init__(self):
        super().__init__('self_sim_publisher')
        self.publisher_ = self.create_publisher(Velocity, '/relay_pose', 1)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.direction_callback,
            10)

        setup()

        self.state = 'stop'
        self.spin = 'stop'

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.subscription
    
    def timer_callback(self):
        msg = Velocity()
        t = 0.0

        if self.state=='forward':

            if self.spin == 'no':
                move(speed_set, self.state, self.spin, 0.0)
                msg.linear_velocity = linear_min
                msg.angular_velocity = 0.0
                t = 1.3

            elif self.spin == 'right':
                move(speed_set, self.state, self.spin, radius_min)
                msg.linear_velocity = linear_min * radius_min
                msg.angular_velocity = -1 * angular_min * radius_min


            elif self.spin == 'left':
                move(speed_set, self.state, self.spin, radius_min)
                msg.linear_velocity = linear_min * radius_min
                msg.angular_velocity = angular_min * radius_min

        elif self.state=='backward':

            if self.spin == 'no':
                move(speed_set, self.state, self.spin, 0.0)
                msg.linear_velocity = -1 * linear_min 
                msg.angular_velocity = 0.0
                t = 0.1

            elif self.spin == 'right':
                move(speed_set, self.state, self.spin, radius_min)
                msg.linear_velocity = -1 * linear_min * radius_min
                msg.angular_velocity = -1 * angular_min * radius_min

            elif self.spin == 'left':
                move(speed_set, self.state, self.spin, radius_min)
                msg.linear_velocity = -1 * linear_min * radius_min
                msg.angular_velocity = angular_min * radius_min


        elif self.state=='no':
            if self.spin == 'right':
                move(speed_set, self.state, self.spin, 0.0)
                msg.angular_velocity = -1 * angular_min 
                msg.linear_velocity = 0.0
                t = 0.13

            elif self.spin=='left':
                move(speed_set, self.state, self.spin, 0.0)
                msg.angular_velocity = angular_min 
                msg.linear_velocity = 0.0
                t = 0.1

        
        elif self.state=='stop':
            motorStop()
            self.linear_velocity = 0.0
            self.linear_angular = 0.0

        self.publisher_.publish(msg)

        time.sleep(t)

        motorStop()

        msg.linear_velocity = 0.0
        msg.angular_velocity = 0.0

        self.publisher_.publish(msg)

        time.sleep(3.0)


    def direction_callback(self, msg):

        if msg.linear.x == 0.0:
            if msg.angular.z == 0.0:
                self.state = 'stop'
                self.spin = 'stop'
            elif msg.angular.z > 0.0:
                self.state = 'no'
                self.spin = 'left'
        
            else :
                self.state = 'no'
                self.spin = 'right'

        elif msg.linear.x > 0.0:
            self.state = 'forward'
            if msg.angular.z == 0.0:
                self.spin = 'no'
            elif msg.angular.z > 0.0:
                self.spin = 'left'
            else:
                self.spin = 'right'
        
        elif msg.linear.x < 0.0:
            self.state = 'backward'
            if msg.angular.z == 0.0:
                self.spin = 'no'
            elif msg.angular.z > 0.0:
                self.spin = 'left'
            else:
                self.spin = 'right'


def main():
    rclpy.init()
    node = SimPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        destroy()
        pass

    rclpy.shutdown()