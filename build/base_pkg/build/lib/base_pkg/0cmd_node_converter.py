import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelConverter(Node):
    def __init__(self):
        super().__init__('cmd_vel_converter')
        
        # Subscriber to /cmd_vel_nav
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel_nav', self.cmd_vel_callback, 10)
        
        # Publisher to /cmd_vel_converted
        self.publisher = self.create_publisher(Twist, '/cmd_vel_converted', 10)
        
        self.get_logger().info("cmd_vel_converter Node Initialized")

    def cmd_vel_callback(self, msg):
        # Directly passing the message (Modify here if transformation is needed)
        self.publisher.publish(msg)
        self.get_logger().info(f"Forwarded cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

