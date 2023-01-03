
import sys 
from rasptank_msgs.srv import Grub
import rclpy
from rclpy.node import Node


class SimpleClient(Node):

    def __init__(self):
        super().__init__('simple_client')
        self.cli = self.create_client(Grub, 'Grub')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Grub().Request()

    def send_request(self):

        self.req.hsv = [0.0, 0.0, 0.0]
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    simple_client = SimpleClient()   
    simple_client.send_request()

    while rclpy.ok():
        
        rclpy.spin_once(simple_client) 
        if simple_client.future.done():
            try:
                response = simple_client.future.result()
            except Exception as e:                                        
                simple_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                simple_client.get_logger().info('Response:name=%s' % response.name)
            break

    simple_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()