import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from example_interfaces.srv import Trigger
import random

class ScannerNode(Node):
    def __init__(self):
        super().__init__('scanner_node')
        self.publisher_ = self.create_publisher(Int32, 'barcode', 10)
        self.last_barcode = 0
        self.timer = self.create_timer(1.0, self.publish_barcode)
        self.srv = self.create_service(Trigger, 'get_barcode', self.get_last_barcode)

    def publish_barcode(self):
        self.last_barcode = random.randint(10000, 99999)
        self.publisher_.publish(Int32(data=self.last_barcode))

    def get_last_barcode(self, request, response):
        response.success = True
        response.message = str(self.last_barcode)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
