import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
from crc8 import crc8
MAX_VELOCITY = 1.0
MAX_OMEGA = 1.0
START_BYTE = 0xA5

RED_TTL = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5XK3RJT-if00-port0'
BLACK_TTL = '/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'
USING_TTL = RED_TTL

class CmdNode(Node):
    def __init__(self):
        super().__init__('cmd_node')
        
        # Subscriber to /cmd_vel_converted
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel_nav', self.cmd_vel_callback, 10)
        
        self.serial_port = serial.Serial(USING_TTL, 115200)
        self.get_logger().info("cmd_node is running...")
    
    def cmd_vel_callback(self, msg):
        # Process and send to STM
        data = [                        
                
            bytes(struct.pack("B", START_BYTE)),
            bytes(struct.pack("f", float(msg.linear.x))),
            bytes(struct.pack("f", float(msg.angular.z)))
        ]
        data = b''.join(data)
        hash_value = self.calc_crc(data[1:])
        data = [data, bytes(struct.pack('B', hash_value))]
        data = b''.join(data)
        self.serial_port.write(data)
        self.get_logger().info(f"Sent to STM: vx={msg.linear.x}, wz={msg.angular.z}")
    
    def calc_crc(self, data=[]):
        hash_func = crc8()
        hash_func.update(data)
        return hash_func.digest()[0]

def main():
    rclpy.init()
    node = CmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

