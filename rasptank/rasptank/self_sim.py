import math
import sys

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist

import numpy as np

import rclpy
from rclpy.node import Node

from .control_modules.move import *

from turtlesim.msg import Pose

from std_msgs.msg import String

linear_min = 0.175 # #your robot min linear velocity [m/s]
angular_min = 0.45 # #your robot min angular velocity [rad/s]
radius_min = 0.3
base_radius = 1.0 - radius_min

linear_interval = 0.015 #your robot linear velocity [m/s]
angular_interval = 0.15  #your robot angular velocity [rad/s]
radius_interval = 0.1

speed_base = 40

class SimPublisher(Node):

    def __init__(self):
        super().__init__('self_sim_publisher')
        self.publisher_ = self.create_publisher(Pose, '/rasptank/pose', 1)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.direction_callback,
            10)

        self.twist = Twist()

        self.state = 'stop'
        self.spin = 'stop'

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        setup()

        self.subscription
    
    def timer_callback(self):
        msg = Pose()

        if self.state=='forward':
            lin_power = self.twist.linear.x-1.0

            if self.spin == 'no':
                move(speed_base+5*lin_power, self.state, self.spin, 0.0)
                self.linear_velocity = linear_min + linear_interval*lin_power
                
                self.angular_velocity = 0.0

            elif self.spin == 'right':
                ang_power = self.twist.angular.z+1.0
                move(speed_base+5*lin_power, self.state, self.spin, radius_min+radius_interval*ang_power)
                t = self.theta
                self.linear_velocity = (linear_min + linear_interval*lin_power)*(base_radius-radius_interval*ang_power)
                self.angular_velocity = (angular_min + -1*angular_interval*ang_power)*(radius_min+radius_interval*ang_power)
                t -= self.timer_period * self.angular_velocity

                if t <= -1*math.pi:
                    t += math.pi * 2 

                self.theta = t

            elif self.spin == 'left':
                ang_power = self.twist.angular.z-1.0
                move(speed_base+5*lin_power, self.state, self.spin, radius_min+radius_interval*ang_power)
                t = self.theta
                self.linear_velocity = (linear_min + linear_interval*lin_power)*(base_radius-radius_interval*ang_power)
                self.angular_velocity = (angular_min + angular_interval*ang_power)*(radius_min+radius_interval*ang_power)
                t += self.timer_period * self.angular_velocity

                if t <= -1*math.pi:
                    t += math.pi * 2 

                self.theta = t

            self.x += self.timer_period * self.linear_velocity * math.cos(self.theta) 
            self.y += self.timer_period * self.linear_velocity * math.sin(self.theta) 

        elif self.state=='backward':
            lin_power = self.twist.linear.x+1.0

            if self.spin == 'no':
                move(speed_base+lin_power*-5.0, self.state, self.spin, 0.0)
                self.linear_velocity = linear_min + -1*linear_interval*lin_power
                self.angular_velocity = 0.0

            elif self.spin == 'right':
                ang_power = self.twist.angular.z+1.0
                move(speed_base+5*lin_power, self.state, self.spin, radius_min+radius_interval*ang_power)
                t = self.theta
                self.linear_velocity = (linear_min + linear_interval*lin_power)*(base_radius-radius_interval*ang_power)
                self.angular_velocity = (angular_min + -1*angular_interval*ang_power)*(radius_min+radius_interval*ang_power)
                t -= self.timer_period * self.angular_velocity

                if t <= -1*math.pi:
                    t += math.pi * 2 

                self.theta = t

            elif self.spin == 'left':
                ang_power = self.twist.angular.z-1.0
                move(speed_base+5*lin_power, self.state, self.spin, radius_min+radius_interval*ang_power)
                t = self.theta
                self.linear_velocity = (linear_min + linear_interval*lin_power)*(base_radius-radius_interval*ang_power)
                self.angular_velocity = (angular_min + angular_interval*ang_power)*(radius_min+radius_interval*ang_power)
                t += self.timer_period * self.angular_velocity

                if t <= -1*math.pi:
                    t += math.pi * 2 

                self.theta = t

            self.x -= self.timer_period * self.linear_velocity * math.cos(self.theta) 
            self.y -= self.timer_period * self.linear_velocity * math.sin(self.theta)

        elif self.state=='no':
            if self.spin == 'right':
                ang_power = self.twist.angular.z+1.0
                move(speed_base+ang_power*-5.0, self.state, self.spin, 0.0)
                t = self.theta

                self.angular_velocity = angular_min + -1*angular_interval*(self.twist.angular.z+1.0)
                t -= self.timer_period * self.angular_velocity

                if t <= -1*math.pi:
                    t += math.pi * 2 

                self.theta = t  
                self.linear_velocity = 0.0

            elif self.spin=='left':
                ang_power = self.twist.angular.z-1.0
                move(speed_base+5.0*ang_power, self.state, self.spin, 0.0)
                t = self.theta
                self.angular_velocity = angular_min + angular_interval*(self.twist.angular.x-1.0)
                t += self.timer_period * self.angular_velocity

                if t > math.pi:
                    t -= math.pi *2

                self.theta = t  
                self.linear_velocity = 0.0

        elif self.state=='stop':
            motorStop()
            self.linear_velocity = 0.0
            self.linear_angular = 0.0

        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta
        msg.linear_velocity = self.linear_velocity
        msg.angular_velocity = self.angular_velocity

        self.publisher_.publish(msg)

    def direction_callback(self, msg):
        self.twist = msg

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
        pass

    rclpy.shutdown()