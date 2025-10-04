from std_msgs.msg import Int32
from std_msgs.msg import Bool
from rclpy.node import Node
import rclpy

class StackLightNode(Node):
    def __init__(self):
        super().__init__('stack_light_node')
        self.pub = self.create_publisher(Int32, 'stack_light', 10)

        self.door_state = True
        self.emergency_state = False

        self.create_subscription(Bool, 'door_state', self.door_cb, 10)
        self.create_subscription(Bool, 'emergency_state', self.emergency_cb, 10)

        self.timer = self.create_timer(1.0, self.publish_status)

    def door_cb(self, msg):
        self.door_state = msg.data

    def emergency_cb(self, msg):
        self.emergency_state = msg.data

    def publish_status(self):
        status = -1 if self.emergency_state else 1 if not self.door_state else 0
        self.pub.publish(Int32(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = StackLightNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()