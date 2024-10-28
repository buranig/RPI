import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32
from ros_g29_force_feedback.msg import ForceFeedback
from bumper_msgs.srv import WheelPosition, JoyBuffer
from sensor_msgs.msg import Joy
from rclpy.qos import qos_profile_sensor_data

import evdev
from evdev import ecodes as e
import time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.joy_buf_req = JoyBuffer.Request()
        self.joy_buf_cli = self.create_client(JoyBuffer, 'joy_buffer')
        while not self.joy_buf_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.joy_pub = self.create_publisher(Joy, '/joy_best', qos_profile_sensor_data)
    
    def joy_buf_request(self):
        self.joy_buf_future = self.joy_buf_cli.call_async(self.joy_buf_req)
        rclpy.spin_until_future_complete(self, self.joy_buf_future)
        if not self.joy_buf_future.done():
            print("Timeout")
        return self.joy_buf_future.result().joy_buffer
    
    def run(self):
        while rclpy.ok():
            joy_msgs = Joy()
            joy_msgs = self.joy_buf_request()

            self.joy_pub.publish(joy_msgs)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    minimal_subscriber.run()
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
