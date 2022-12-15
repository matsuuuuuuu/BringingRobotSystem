import sys
import time
import math

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
F = 550
z = 0
Z = 228.5 #物体-カメラ間の距離

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

class State(IntEnum):
    INIT = auto()
    WAIT = auto()
    DETECT = auto()
    SEND = auto()
    ERROR = auto()

def calc_direction(pt_R, z):
    x = (2*z*pt_R-Width_*z)/(2*F*100) - camera_pos[1]
    y = z/100 - camera_pos[0]
    if nav_camera == Direction.FLONT:
        return x*(-1), y*(-1)
    elif nav_camera == Direction.BACK:
        return x, y
    elif nav_camera == Direction.RIGHT:
        return y*(-1), x
    elif nav_camera == Direction.LEFT:
        return y, x*(-1)

class ObjectDetector(Node):

    def __init__(self):
        self.rect = [[0] * 2 for i in range(2)]
        self.object_name = ''

        self.state = State.INIT

        super().__init__('Object_Detector')
        self.srv = self.create_service(Detect, 'detection', self.service_callback)

        self.cli_carry = self.create_client(Carry, 'carrying')
        while not self.cli_carry.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('carry service not available, waiting again...')
        self.request_carry = Carry.Request()
        #self.request_carry.pos = [[0.0] * 2 ]
        #self.request_carry.hsv = [[0.0] * 3 ]

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

        self.state = State.WAIT

    #def print_state(self):
    #    print(self.state)

    def service_callback(self, request, response):
        self.state = State.DETECT
        self.object_name = request.name
        self.request_carry.dest = request.dest
        print(self.state)
        response.res = 1

        return response

    def calc_position(self):

        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (Width_,Height_), 1, (Width_,Height_))
        img_R = self.loadimg_R()
        blob = cv.dnn.blobFromImage(img_R, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)

        # determine the output layer
        ln = self.net.getLayerNames()
        ln = [ln[i - 1] for i in self.net.getUnconnectedOutLayers()] 

        outputs = self.net.forward(ln)

        # combine the 3 output groups into 1 (10647, 85)
        # large objects (507, 85)
        # medium objects (2028, 85)
        # small objects (8112, 85)
        outputs = np.vstack(outputs)   

        dst_img_R = cv.undistort(img_R, mtx, dist, None, newcameramtx)
        cv.imwrite('/home/enpit/Pictures/dst_img_R.jpg', dst_img_R)
        #time_sta = time.time() #timer start
        success, hsv_R = self.post_process(dst_img_R, outputs, 0.5, self.object_name)
        #time_end = time.time() # timer stop
        #print('time: "%5f"' , time_end-time_sta)

        if success == False:
            #response.res = 'dection fault'
            self.state = State.WAIT
            return 

        imgBox = dst_img_R[self.rect[0][1]: self.rect[1][1], self.rect[0][0]: self.rect[1][0]]
        pt_R = self.outline_object(imgBox, hsv_R)
        print(self.rect[0][0])
        pt_R += self.rect[0][0]
        diff_R = self.rect[1][0] - self.rect[0][0]

        img_L = self.loadimg_L()
        blob = cv.dnn.blobFromImage(img_L, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)

        ln = self.net.getLayerNames()
        ln = [ln[i - 1] for i in self.net.getUnconnectedOutLayers()]

        outputs = self.net.forward(ln)
        outputs = np.vstack(outputs)  

        dst_img_L = cv.undistort(img_L, mtx, dist, None, newcameramtx)
        cv.imwrite('/home/enpit/Pictures/dst_img_L.jpg', dst_img_L)
        success, hsv_L = self.post_process(dst_img_L, outputs, 0.5, self.object_name)

        if success == False:
            #response.res = 'detection fault'
            self.state = State.WAIT
            return

        
        imgBox = dst_img_L[self.rect[0][1]: self.rect[1][1], self.rect[0][0]: self.rect[1][0]]
        pt_L = self.outline_object(imgBox, hsv_R)
        print(self.rect[0][0])
        pt_L += self.rect[0][0]
        diff_L = self.rect[1][0] - self.rect[0][0]

        
        print("pt_R:", pt_R, ", pt_L:", pt_L)
        '''
        print(diff_R, diff_L)
        diff = diff_R - diff_L
        if(diff<0):
            pt_L += diff/2
        else:
            pt_R += diff/2
        '''

        self.request_carry.hsv = [(hsv_R[i]+hsv_L[i])/2 for i in range(len(hsv_R))]


        print("pt_R:", pt_R, ", pt_L:", pt_L)
        D = pt_L - pt_R
        f = Z*D/T
        z = F*T/D

        zx = math.sqrt(z*z-camera_height*camera_height)

        self.request_carry.pos = calc_direction(pt_R, zx)

        print("X:", self.request_carry.pos[0], ", Y:", self.request_carry.pos[1])

        #if pt_R > 500:
        #    z += 100
        
        print('F: ', f)
        print('Z: ', z)

        #response.res = 'detection success'

        self.state = State.SEND
        
    #Right camera image load
    def loadimg_R(self):

        self.request_frame.device = 'Right'

        future = self.cli_frame.call_async(self.request_frame)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        img_R = self.bridge.imgmsg_to_cv2(response.frame, "bgr8")
        

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

    def outline_object(self, img, hsv):
        #画像の変数宣言
        #pic_name="mikan"+str(i+1)+ ".jpg"
        pic_name_out="Test_out.jpg"
        gray_pic="Test_gray.jpg"
        #画像のサイズ情報取得
        height, width = img.shape[:2]
        imghsv = cv.cvtColor(img, cv.COLOR_BGR2HSV_FULL)
        h = imghsv[:, :, 0]
        s = imghsv[:, :, 1]
        v = imghsv[:, :, 2]
        #ベース画像と同じ大きさの配列を作成
        img_mikan=np.zeros((height,width,3),np.uint8)
        #色を指定
        img_mikan[(h < hsv[0]+5) & (h > hsv[0]-5) & (s > hsv[1]-5)& (v > hsv[2]-10)] = 255
        #指定色の領域だけの画像を作成
        cv.imwrite(gray_pic,np.array(img_mikan))
        #作成した画像を読み込み
        img_gray = cv.imread(gray_pic,cv.IMREAD_GRAYSCALE)
        #読み込んだ画像の重心、輪郭を取得
        M = cv.moments(img_gray, False)
        contours, hierarchy= cv.findContours(img_gray, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        x,y= int(M["m10"]/M["m00"]) , int(M["m01"]/M["m00"])
        #ベース画像に重心、輪郭追加して保存
        cv.circle(img, (x,y), 20, 100, 2, 4)
        cv.drawContours(img, contours, -1, color=(0, 0, 0), thickness=5)
        print(x,y)
        cv.imwrite(pic_name_out,np.array(img))

        return x
    
    def post_process(self, img, outputs, conf, name):
        boxes = []
        confidences = []
        classIDs = []

        hsv = []
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

                    imgBox = img[self.rect[0][1]+cuty: self.rect[1][1]-cuty, self.rect[0][0]+cutx: self.rect[1][0]-cutx]
                    cv.imwrite("kiridasi.jpg",np.array(imgBox))
                    self.rect[0][0] -= cutx
                    self.rect[1][0] += cutx
                    self.rect[0][1] -= cuty
                    self.rect[1][1] += cuty

                    imgBoxHsv = cv.cvtColor(imgBox,cv.COLOR_BGR2HSV_FULL)
                    hsv.insert(0,imgBoxHsv.T[0].flatten().mean())
                    hsv.insert(1,imgBoxHsv.T[1].flatten().mean())
                    hsv.insert(2,imgBoxHsv.T[2].flatten().mean())

                    print('H:',hsv[0], ', S:', hsv[1], ', V:', hsv[2])
                    success = True
                    return success, hsv

                #print(self.classes[classIDs[i]])
        return success, hsv

    def send_carryrequest(self):

        future = self.cli_carry.call_async(self.request_carry)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        #img_R = cv.imread("/home/enpit/pic/room_R.jpg")
        self.state = State.WAIT
        
        return 


def main(args=None):
    rclpy.init(args=args)

    objectdetector = ObjectDetector()
        
    while rclpy.ok(): 
        
        if objectdetector.state == State.WAIT:
            rclpy.spin_once(objectdetector)

        elif objectdetector.state == State.DETECT:
            objectdetector.calc_position()

        elif objectdetector.state == State.SEND:
            print('state : SEND')
            objectdetector.send_carryrequest()    

        time.sleep(0.1)


    rclpy.shutdown()

    cv.destroyAllWindows()

if __name__ =='__main__':
    main()