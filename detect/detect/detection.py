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

T = 7   #2つのカメラのレンズ間距離
F = 450 #焦点距離
Z = 225.7 #物体-カメラ間の距離

#カメラキャリブレーションに必要な情報
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


#地面からカメラまでの高さ[cm]
camera_height = 73.7

#map上の右カメラの座標(カメラの置き場所によって変更) [x,y]
camera_pos = [-0.2, 1.2]

class Direction(IntEnum):
    FLONT = auto()
    BACK = auto()
    RIGHT = auto()
    LEFT = auto()

nav_camera = Direction.FLONT

#カメラのピクセル数
Width_ = 640
Height_ = 480 

#色を抽出する範囲（値を大きくするほど範囲は狭くなる）
cut = 2.5

class State(IntEnum):
    INIT = auto()
    WAIT = auto()
    DETECT = auto()
    SEND = auto()
    ERROR = auto()

#カメラの置き場所に応じて物体の位置を算出する
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

        self.state = State.INIT

        super().__init__('Detect_system')
        self.srv = self.create_service(Detect, 'detection', self.recieve_name_callback)

        self.publisher_ = self.create_publisher(Int8, 'result', 10)
        self.msg = Int8()

        self.cli_carry = self.create_client(Carry, 'carrying')
        while not self.cli_carry.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('carry service not available, waiting again...')
        self.request_carry = Carry.Request()

        self.cli_frame = self.create_client(Frame, 'camera')
        while not self.cli_frame.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('camera service not available, waiting again...')
        self.request_frame = Frame.Request()
        self.bridge = CvBridge()


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

    #エラーを探し物名入力システムに送信する
    def publish_result(self):
        self.publisher_.publish(self.msg)

    #探し物名入力システムから探し物名と届け先の座標を受信する
    def recieve_name_callback(self, request, response):
        if(self.state == State.WAIT):
            response.res = True
        else:
            response.res = False
            return response

        self.state = State.DETECT
        self.object_name = request.name
        self.request_carry.dest = request.dest
        print(self.state)
        return response

    #探し物の位置推定を行う
    def calc_position(self):
        
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (Width_,Height_), 1, (Width_,Height_))

        #右カメラの撮影をcamera_serverに依頼
        img_R = self.loadimg_R()

        #撮影した写真に対して歪み補正を行う
        dst_img_R = cv.undistort(img_R, mtx, dist, None, newcameramtx)

        #time_sta = time.time() #timer start

        #撮影した写真から物体の検出
        blob = cv.dnn.blobFromImage(dst_img_R, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        ln = self.net.getLayerNames()
        ln = [ln[i - 1] for i in self.net.getUnconnectedOutLayers()] 
        outputs = self.net.forward(ln)
        outputs = np.vstack(outputs)   

        #検出された物体名の中から入力された探し物名と一致する物を見つけ出す
        #色の抽出
        success, hsv_R = self.post_process(dst_img_R, outputs, 0.5, self.object_name)

        #time_end = time.time() # timer stop
        #print('time: "%5f"' , time_end-time_sta)

        #一致する物体がなかった場合
        if success == False:
            self.state = State.ERROR
            return 

        imgBox = dst_img_R[self.rect[0][1]: self.rect[1][1], self.rect[0][0]: self.rect[1][0]]
        pt_R = (self.rect[0][0]+self.rect[1][0])/2

        #左カメラの撮影をcamera_serverに依頼
        img_L = self.loadimg_L()

        #撮影した写真に対して歪み補正を行う
        dst_img_L = cv.undistort(img_L, mtx, dist, None, newcameramtx)

        #撮影した写真から物体の検出
        blob = cv.dnn.blobFromImage(dst_img_L, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        ln = self.net.getLayerNames()
        ln = [ln[i - 1] for i in self.net.getUnconnectedOutLayers()]
        outputs = self.net.forward(ln)
        outputs = np.vstack(outputs)  

        #検出された物体名の中から入力された探し物名と一致する物を見つけ出す
        #色の抽出
        success, hsv_L = self.post_process(dst_img_L, outputs, 0.5, self.object_name)
        pt_L = (self.rect[0][0]+self.rect[1][0])/2

        #一致する物体がなかった場合
        if success == False:
            self.state = State.ERROR
            return

        
        imgBox = dst_img_L[self.rect[0][1]: self.rect[1][1], self.rect[0][0]: self.rect[1][0]]

        #抽出した色を"/carry"のメッセージ用にまとめる
        self.request_carry.hsv[0] = hsv_L[0]
        self.request_carry.hsv[1] = hsv_L[1]
        self.request_carry.hsv[2] = hsv_L[2]

        #探し物の距離を推定する
        D = pt_L - pt_R
        f = Z*D/T
        z = F*T/D
        zx = math.sqrt(z*z-camera_height*camera_height)

        #探し物の位置を推定する
        self.request_carry.pos = calc_direction(pt_R, zx)

        print("X:", self.request_carry.pos[0], ", Y:", self.request_carry.pos[1])

        #if pt_R > 500:
        #    z += 100
        
        #print('F: ', f)
        #print('Z: ', z)

        #response.res = 'detection success'

        self.state = State.SEND

        return 
        
    #右カメラの撮影依頼を行うメソッド
    def loadimg_R(self):

        self.request_frame.device = 'Right'

        #"/camera"サービスに部屋写真撮影依頼を行う
        future = self.cli_frame.call_async(self.request_frame)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        img_R = self.bridge.imgmsg_to_cv2(response.frame, "bgr8")

        print('success')
        

        #img_R = cv.imread("/home/enpit/pic/room_R.jpg")
        
        return img_R

    #左カメラの撮影依頼を行うメソッド
    def loadimg_L(self):
        
        self.request_frame.device = 'Left'
    
        #"/camera"サービスに部屋写真撮影依頼を行う
        future = self.cli_frame.call_async(self.request_frame)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()

        img_L = self.bridge.imgmsg_to_cv2(response.frame, "bgr8")
        
        #img_L = cv.imread("/home/enpit/pic/room_L.jpg")

        return img_L
    
    #検出された物体の中に入力された探し物名と一致するものが存在するか調べるメソッド
    def post_process(self, img, outputs, conf, name):

        #検出された物体の画像上の位置を表す四角
        boxes = []
        #物体の確信度（%）
        confidences = []
        #検出さらた物体のラベル名
        classIDs = []

        hsv = [0*3]
        success = False

        H, W = img.shape[:2]

        #検出された物体の情報をそれぞれの配列に代入する
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
                #検出された物体名と入力の探し物名が一致した場合
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

                    if hsv[0]>180:
                        hsv[0] = hsv[0]-180

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
        #検知依頼待機状態
        if objectdetector.state == State.WAIT:
            rclpy.spin_once(objectdetector)

        #探し物検知状態
        elif objectdetector.state == State.DETECT:
            objectdetector.calc_position()

        #運搬依頼送信状態
        elif objectdetector.state == State.SEND:
            print('state : SEND')
            objectdetector.send_carryrequest()   

        #エラー送信状態
        elif objectdetector.state == State.ERROR:
            objectdetector.msg.data = 11
            objectdetector.publish_result() 
            objectdetector.state = State.WAIT

        time.sleep(0.1)


    rclpy.shutdown()

    cv.destroyAllWindows()

if __name__ =='__main__':
    main()