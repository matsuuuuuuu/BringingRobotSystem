import math
import sys

import numpy as np

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from rasptank_msgs.msg import Velocity


class RelaySim(Node):

    def __init__(self):
        super().__init__('self_sim_relay')
        self.publisher_ = self.create_publisher(Pose, '/rasptank/pose', 1)
        self.subscription = self.create_subscription(
            Velocity,
            '/relay_pose',
            self.velocity_callback,
            1)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.subscription
    
    def timer_callback(self):
        msg = Pose()

        t = self.theta
        t += self.timer_period * self.angular_velocity

        if t <= -1*math.pi:
              t += math.pi * 2 
        elif t > math.pi:
              t -= math.pi *2

        self.theta = t

        self.x += self.timer_period * self.linear_velocity * math.cos(self.theta) 
        self.y += self.timer_period * self.linear_velocity * math.sin(self.theta) 

        linear_v = self.linear_velocity

        if self.linear_velocity < 0.0:
            linear_v *= -1

        angular_v = self.linear_velocity
        if self.angular_velocity < 0.0:
            angular_v *= -1

        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta
        msg.linear_velocity = linear_v
        msg.angular_velocity = angular_v

        self.publisher_.publish(msg)


    def velocity_callback(self, msg):
        self.linear_velocity = msg.linear_velocity
        self.angular_velocity = msg.angular_velocity

def main():
    rclpy.init()
    node = RelaySim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()