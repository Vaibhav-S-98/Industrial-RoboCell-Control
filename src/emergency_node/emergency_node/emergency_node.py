import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from example_interfaces.srv import Trigger
from std_msgs.msg import Bool
import random

class EmergencyNode(Node):
    def __init__(self):
        super().__init__('emergency_node')
        self.state = False
        self.pub = self.create_publisher(Bool, 'emergency_state', 10)
        self.timer = self.create_timer(1.0, self.publish)
        self.srv = self.create_service(Trigger, 'get_emergency_state', self.handle_request)

    def publish(self):
        self.pub.publish(Bool(data=self.state))

    def handle_request(self, request, response):
        response.success = self.state
        response.message = "Emergency Active" if self.state else "Normal"
        return response

    
def main(args=None):
    rclpy.init(args=args)
    node = EmergencyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
