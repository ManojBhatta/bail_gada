import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class TopicRelay(Node):
    def __init__(self):
        super().__init__('topic_relay')
        self.subscription = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(Odometry, '/odom', 10)

    def odom_callback(self, msg):
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TopicRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
