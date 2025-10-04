import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from example_interfaces.srv import Trigger
import random

class DoorNode(Node):
    def __init__(self):
        super().__init__('door_node')
        self.state = True
        self.pub = self.create_publisher(Bool, 'door_state', 10)
        self.timer = self.create_timer(1.0, self.publish)
        self.srv = self.create_service(Trigger, 'get_door_state', self.handle_request)

    def publish(self):
        self.pub.publish(Bool(data=self.state))

    def handle_request(self, request, response):
        response.success = self.state
        response.message = "Door Closed" if self.state else "Door Open"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DoorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
