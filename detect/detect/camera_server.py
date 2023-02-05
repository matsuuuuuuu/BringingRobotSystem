import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError

from my_interfaces.srv import Frame


class CameraServer(Node):

    def __init__(self):

        super().__init__('Camera_Server')

        self.srv = self.create_service(Frame, 'camera', self.send_frame)

        #self.cap_L = cv2.VideoCapture(4)
        #self.cap_R = cv2.VideoCapture(2)

        self.bridge = CvBridge()

    #入力に応じて左右のカメラで撮影を行う
    def send_frame(self, request, response):
        #右側
        if request.device == 'Right':
            cap_R = cv2.VideoCapture(2)
            print('Right')
            ret, frame = cap_R.read()
            cap_R.release()
        #左側
        elif request.device == 'Left':
            cap_L = cv2.VideoCapture(4)
            print('Left')
            ret, frame = cap_L.read()
            cap_L.release()

        #写真の撮影が成功したかを調べる
        try:
            #写真の送信
            response.frame = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            print('send_frame')

        except CvBridgeError as e:
           print('e')

        return response

def main(args=None):
    rclpy.init(args=args)

    cameraserver = CameraServer()
    rclpy.spin(cameraserver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        
