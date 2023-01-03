import sys
import time
import rclpy
from rclpy.node import Node
from enum import IntEnum, auto

from geometry_msgs.msg import Twist
from rasptank_msgs.srv import Grub, Release

import numpy as np

import cv2

from .control_modules.RPIservo import *

WIDTH = 640
HEIGHT = 480

ksize = 9

speed_set = 40


scGear = ServoCtrl()
scGear.moveInit()
scGear.start()

init_pwm0 = scGear.initPos[0]
init_pwm1 = scGear.initPos[1]
init_pwm2 = scGear.initPos[2]
init_pwm3 = scGear.initPos[3]


#manage state
class State(IntEnum):
    INIT = auto()
    WAIT = auto()
    GRUB = auto()
    RELEASE = auto()
    ERROR = auto()

#init servo position 
def servoPosInit():
    scGear.initConfig(0,init_pwm0,1)
    scGear.initConfig(1,init_pwm1,1)
    scGear.initConfig(2,init_pwm2,1)
    scGear.initConfig(3,init_pwm3,1)

#motion of grub
def grub():
    scGear.singleServo(15, -1, 1)
    time.sleep(1.0)
    scGear.singleServo(13, 1, 3)
    time.sleep(1.0)
    scGear.singleServo(15, 1, 1)
    time.sleep(2.0)
    scGear.singleServo(13, -1, 1)
    time.sleep(2.5)
    print("grub")

#motion of grub
def release():
    scGear.singleServo(13, 1, 3)
    time.sleep(1.0)
    scGear.singleServo(15, -1, 1)
    time.sleep(1.0)
    scGear.singleServo(13, -1, 1)
    time.sleep(2.5)
    print("release")



class ColorTracking(Node):
    def __init__(self):
        self.state = State.INIT
        super().__init__(node_name = 'grub_object')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        
        self.srv_grub = self.create_service(Grub, 'grub', self.grub_callback)
        self.srv_release = self.create_service(Release, 'release', self.release_callback)
        self.hsv = []
        
        self.count_ = 0
        
        self.state = State.WAIT
        
    #when recieve /release request
    def release_callback(self, request, response):
        self.state = State.RELEASE
        release()
        response.res = True
        self.state = State.WAIT
        return response
        
    #when recieve /grub request    
    def grub_callback(self, request, response):
        self.state = State.GRUB
        self.hsv = request.hsv
        self.hsv[0] = int((self.hsv[0]/255)*180)
        print("self:", self.hsv[0])
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        success = False
        t =0.0
        count = 0
        
        while count<7:
            while True:
                #take a picture
                img = self.get_frame()
                #calc centor of gravity of value of hsv
                x, y = self.calc_outline(img, self.hsv)
                
                print(x)
                print(y)
                success_x = False
                success_y = False
                if x == -1:
                    break
                elif x < (WIDTH/2)-5:
                    #turn left
                    msg.angular.z = 0.3
                    t = 0.13
                elif x > (WIDTH/2)+5:
                    #turn right
                    msg.angular.z = -0.3
                    t = 0.1
                else:
                    print('x: success')
                    success_x = True

                self.publisher_.publish(msg)
                time.sleep(t)
            
                msg.angular.z = 0.0  
                self.publisher_.publish(msg)
                time.sleep(1.0)
            
                if y == -1:
                    break
                if y < HEIGHT*2/3+20:
                    #move forward
                    msg.linear.x = 0.1
                    t = 0.13
                elif y > HEIGHT*2/3+30 :
                    #move backward
                    msg.linear.x = -0.1
                    t = 0.1
                else:
                    print('y: success')
                    success_y = True
                
                if success_x&success_y:
                    success = True
                    break
                
                self.publisher_.publish(msg)
                time.sleep(t)
            
                msg.linear.x = 0.0 
                self.publisher_.publish(msg)
                time.sleep(1.0)
                
                

            if success:
                print('success')
                grub()
                break
                
            #if don't exist object, turn right 
            msg.angular.z = -0.3
            self.publisher_.publish(msg)
            time.sleep(3.0)
            msg.angular.z = 0.0  
            self.publisher_.publish(msg)
            time.sleep(1.0)
            count += 1
            
        self.state = State.WAIT
        if success:
            response.res = True
        else:
            response.res = False
        
        return response
    
    #function of taking a picture by a camera module
    def get_frame(self):
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        self.count_ += 1
        name = 'get_pic' + str(self.count_) + '.jpg' 
        cv2.imwrite(name, np.array(frame))
        print("img_saved!")
        cap.release()
        
        return frame
        
    def calc_outline(self, img, hsv):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV_FULL)
        #print(img_hsv)
        h = img_hsv[:, :, 0]
        s = img_hsv[:, :, 1]
        v = img_hsv[:, :, 2]
        
        #img_copy = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        #print(img_copy)
        
        h_max = hsv[0]+10
        h_min = hsv[0]-10
        
        if h_max>255:
            h_max = 255
        elif h_min<0:
            h_min = 0
            
        s_max = hsv[1]+100
        s_min = hsv[1]-100
        
        if s_max>255:
            s_max = 255
        elif s_min<0:
            s_min = 0
            
        v_max = hsv[2]+100
        v_min = hsv[2]-100
        
        if v_max>255:
            v_max = 255
        elif v_min<0:
            v_min = 0
        #hsv_min = (int(h_min), 0, 0)
        #hsv_max = (int(h_max), 255, 255)
        hsv_max = (int(h_max), int(s_max), int(v_max))
        hsv_min = (int(h_min), int(s_min), int(v_min))
        
        img_gray = cv2.inRange(img_hsv, hsv_min, hsv_max)    # HSV‚©‚çƒ}ƒXƒN‚ðì¬
        img_mask = cv2.medianBlur(img_gray,ksize)
        cv2.imwrite('gray_pic.jpg', np.array(img_gray))
        cv2.imwrite('mask_pic.jpg', np.array(img_mask))
        
        M = cv2.moments(img_mask, False)
        #contours, hierarchy = cv2.findContours(img_gray, cv2. RETR_EXTERNAL, cv2.CHIN_APPROX_SIMPLE)
        
        if M["m00"]==0:
          x = -1
          y = -1
          return x,y
        else:
          x,y = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
          return x, y

def main(args=None):
    rclpy.init(args=args)
    
    servoPosInit()
    print("set init pos")
    
    colortracking = ColorTracking()

    while rclpy.ok():
        if colortracking.state == State.WAIT:
            rclpy.spin_once(colortracking)
        time.sleep(0.1)

    
    
    rclpy.shutdown()
