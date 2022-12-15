import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge, CvBridgeError

from my_interfaces.srv import Frame


class CameraServer(Node):

    def __init__(self):

        super().__init__('Camera_Server')

        self.srv = self.create_service(Frame, 'camera', self.send_frame)

        self.cap_L = cv2.VideoCapture(4)
        self.cap_R = cv2.VideoCapture(2)

        self.bridge = CvBridge()

    def send_frame(self, request, response):
        if request.device == 'Right':
            print('Right')
            ret, frame = self.cap_R.read()
        elif request.device == 'Left':
            print('Left')
            ret, frame = self.cap_L.read()


        try:
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
        
