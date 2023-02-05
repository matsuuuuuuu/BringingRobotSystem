import sys
import time
import math

from std_msgs.msg import Int8
from enum import IntEnum, auto
# YOLO object detection
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from my_interfaces.srv import Frame, Detect, Carry
import rclpy
from rclpy.node import Node

T = 7
F = 400
Z = 172 #物体-カメラ間の距離

fx = 709.11284991
fy = 704.88920349
cx = 335.27842056
cy = 222.35231474

mtx = np.array([[fx, 0, cx], [0, fx, cy], [0, 0, 1]])

k1 = 0.0323993578
k2 = 0.764621312
p1 = -0.0069192255
p2 = -0.0067504193
k3 = -2.37697268

dist = np.array([[k1, k2, p1, p2, k3]])

#地面からカメラまでの距離[cm]
camera_height = 74.7

#map上の右カメラの座標(カメラの置き場所によって変更) [x,y]
camera_pos = [0.0, 1.0]

class Direction(IntEnum):
    FLONT = auto()
    BACK = auto()
    RIGHT = auto()
    LEFT = auto()

nav_camera = Direction.FLONT

Width_ = 640
Height_ = 480 

cut = 2.5


def calc_direction(pt_R, z):
    x = (2*z*pt_R-Width_*z)/(2*F*100) - camera_pos[0]
    y = z/100 - camera_pos[1]
    print
    if nav_camera == Direction.FLONT:
        return x*(-1)-0.1, y*(-1)
    elif nav_camera == Direction.BACK:
        return x-0.1, y
    elif nav_camera == Direction.RIGHT:
        return y*(-1)-0.1, x
    elif nav_camera == Direction.LEFT:
        return y-0.1, x*(-1)

class ObjectDetector(Node):

    def __init__(self):
        self.rect = [[0] * 2 for i in range(2)]
        self.object_name = ''

        super().__init__('setup_stereocamera')

        self.cli_frame = self.create_client(Frame, 'camera')
        while not self.cli_frame.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('camera service not available, waiting again...')
        self.request_frame = Frame.Request()
        self.bridge = CvBridge()

        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.print_state)

        # Load names of classes and get random colors
        self.classes = open('/home/enpit/yolo/darknet/cfg/color_ball.names').read().strip().split('\n')
        np.random.seed(42)
        self.colors = np.random.randint(0, 255, size=(len(self.classes), 3), dtype='uint8')

        # Give the configuration and weight files for the model and load the network.
        self.net = cv.dnn.readNetFromDarknet('/home/enpit/yolo/darknet/cfg/color_ball_test.cfg', '/home/enpit/yolo/darknet/data/color_ball/backup/color_ball_train_final.weights')
        self.net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
        # net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)

    #探し物の位置推定を行う
    def calc_position(self):
        
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (Width_,Height_), 1, (Width_,Height_))

        #右カメラの撮影をcamera_serverに依頼
        img_R = self.loadimg_R()
        dst_img_R = cv.undistort(img_R, mtx, dist, None, newcameramtx)

        blob = cv.dnn.blobFromImage(dst_img_R, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)

        # determine the output layer
        ln = self.net.getLayerNames()
        ln = [ln[i - 1] for i in self.net.getUnconnectedOutLayers()] 

        outputs = self.net.forward(ln)
        outputs = np.vstack(outputs)   

        
        cv.imwrite('/home/enpit/Pictures/dst_img_R.jpg', dst_img_R)

        #time_sta = time.time() #timer start

        #物体の検出、色の抽出
        success, hsv_R = self.post_process(dst_img_R, outputs, 0.5, self.object_name)
        #time_end = time.time() # timer stop
        #print('time: "%5f"' , time_end-time_sta)

        if success == False:
            print("system didn't detect object in right pic")
            return 

        imgBox = dst_img_R[self.rect[0][1]: self.rect[1][1], self.rect[0][0]: self.rect[1][0]]
        pt_R = (self.rect[0][0]+self.rect[1][0])/2
        diff_R = self.rect[1][0] - self.rect[0][0]

        #左カメラの撮影をcamera_serverに依頼
        img_L = self.loadimg_L()
        dst_img_L = cv.undistort(img_L, mtx, dist, None, newcameramtx)
        blob = cv.dnn.blobFromImage(dst_img_L, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)

        ln = self.net.getLayerNames()
        ln = [ln[i - 1] for i in self.net.getUnconnectedOutLayers()]

        outputs = self.net.forward(ln)
        outputs = np.vstack(outputs)  


        cv.imwrite('/home/enpit/Pictures/dst_img_L.jpg', dst_img_L)
        success, hsv_L = self.post_process(dst_img_L, outputs, 0.5, self.object_name)

        if success == False:
            print("system didn't detect object in left pic")
            return

        
        imgBox = dst_img_L[self.rect[0][1]: self.rect[1][1], self.rect[0][0]: self.rect[1][0]]
        pt_L = (self.rect[0][0]+self.rect[1][0])/2
        diff_L = self.rect[1][0] - self.rect[0][0]

        print("diff_R:", diff_R, ", diff_L", diff_L)
        print("pt_R:", pt_R, ", pt_L:", pt_L)

        print(diff_R, diff_L)
        '''
        diff = diff_R - diff_L
        if(diff<0):
            pt_L -= diff/2
        else:
            pt_R -= diff/2
        '''

        D = pt_L - pt_R
        #f = Z*D/T
        z = F*T/D

        #zx = math.sqrt(z*z-camera_height*camera_height)

        #request_carry.pos = calc_direction(pt_R, zx)

        #print("X:", request_carry.pos[0], ", Y:", request_carry.pos[1])

        #if pt_R > 500:
        #    z += 100
        
        #print('F: ', f)
        print('Z: ', z)

        #response.res = 'detection success'


        return 
        
    #Right camera image load
    def loadimg_R(self):

        self.request_frame.device = 'Right'

        future = self.cli_frame.call_async(self.request_frame)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        img_R = self.bridge.imgmsg_to_cv2(response.frame, "bgr8")

        print('success')
        

        #img_R = cv.imread("/home/enpit/pic/room_R.jpg")
        
        return img_R

    #Left camera image load
    def loadimg_L(self):
        
        self.request_frame.device = 'Left'
    
        future = self.cli_frame.call_async(self.request_frame)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        img_L = self.bridge.imgmsg_to_cv2(response.frame, "bgr8")
        
        #img_L = cv.imread("/home/enpit/pic/room_L.jpg")

        return img_L
    
    def post_process(self, img, outputs, conf, name):
        boxes = []
        confidences = []
        classIDs = []

        hsv = [0*3]
        success = False

        H, W = img.shape[:2]

        for output in outputs:
            scores = output[5:]
            classID = np.argmax(scores)
            confidence = scores[classID]
            if confidence > conf:
                x, y, w, h = output[:4] * np.array([W, H, W, H])
                p0 = int(x - w//2), int(y - h//2)
                p1 = int(x + w//2), int(y + h//2)
                boxes.append([*p0, int(w), int(h)])
                confidences.append(float(confidence))
                classIDs.append(classID)

        indices = cv.dnn.NMSBoxes(boxes, confidences, conf, conf-0.1)

        if len(indices) > 0:
            for i in indices.flatten():
                if self.classes[classIDs[i]] == name:

                    print('success')

                    self.rect[0][0] = boxes[i][0]
                    self.rect[0][1] = boxes[i][1]
                    self.rect[1][0] = boxes[i][0]+boxes[i][2]
                    self.rect[1][1] = boxes[i][1]+boxes[i][3]
                    
                    cutx = int((self.rect[1][0]-self.rect[0][0])/cut)
                    cuty = int((self.rect[1][1]-self.rect[0][1])/cut)

                    #色を抽出するために写真から物体の中心部分の切り抜き
                    imgBox = img[self.rect[0][1]+cuty: self.rect[1][1]-cuty, self.rect[0][0]+cutx: self.rect[1][0]-cutx]
                    cv.imwrite("kiridasi.jpg",np.array(imgBox))
                    self.rect[0][0] -= cutx
                    self.rect[1][0] += cutx
                    self.rect[0][1] -= cuty
                    self.rect[1][1] += cuty

                    #切り抜いた部分のHSV値の平均値を算出
                    imgBoxHsv = cv.cvtColor(imgBox,cv.COLOR_BGR2HSV_FULL)
                    hsv.insert(0,int(imgBoxHsv.T[0].flatten().mean()))
                    hsv.insert(1,int(imgBoxHsv.T[1].flatten().mean()))
                    hsv.insert(2,int(imgBoxHsv.T[2].flatten().mean()))

                    print('H:',hsv[0], ', S:', hsv[1], ', V:', hsv[2])
                    success = True
                    return success, hsv

                #print(self.classes[classIDs[i]])
        return success, hsv



def main(args=None):
    rclpy.init(args=args)

    objectdetector = ObjectDetector()
        
    objectdetector.object_name = input("Name:")

    objectdetector.calc_position()

    rclpy.shutdown()

    cv.destroyAllWindows()

if __name__ =='__main__':
    main()