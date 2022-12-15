import sys
import time

from enum import IntEnum, auto
from my_interfaces.srv import Detect
import rclpy
from rclpy.node import Node

dest_ = [[-1.2, 0.8], [1.0, 2.0]]

class State(IntEnum):
    INIT = auto()
    INPUT = auto()
    WAIT = auto()
    SEND = auto()
    ERROR = auto()

class MinimalClientAsync(Node):

    def __init__(self):
        self.state = State.INIT
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Detect, 'detection')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Detect.Request()
        self.state = State.INPUT


    def send_request(self, name, dest):
        self.req.name = name
        self.req.dest = dest_[0]
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    
    while rclpy.ok():
        if minimal_client.state == State.INPUT:
            Name = input("探し物名:")
            Dest = input("届け先:")
            minimal_client.state = State.SEND

        elif minimal_client.state == State.SEND:
            response = minimal_client.send_request(Name, Dest)
            minimal_client.state = State.INPUT
            print('result:', response.res)
      
        if minimal_client.state == State.WAIT:
            rclpy.spin_once(minimal_client) 

        time.sleep(0.1)

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
