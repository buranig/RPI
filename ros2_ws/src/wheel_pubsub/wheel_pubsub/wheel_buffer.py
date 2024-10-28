#!/usr/bin/env python3
    
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from bumper_msgs.srv import WheelPosition, JoyBuffer

class WheelBuffer(Node):
    def __init__(self):
        super().__init__('wheel_buffer')

        self.wheel_sub_ = self.create_subscription(Joy, '/joy', self._wheel_state, qos_profile_sensor_data)
          
        self.wheel_service = self.create_service(WheelPosition, 'wheel_buffer', self._get_env_state)
        self.wheel_goal = self.create_service(WheelPosition, 'wheel_goal', self._get_goal_position)
        self.joy_msgs_buf = self.create_service(JoyBuffer, 'joy_buffer', self._get_joy_msg_buf)
        
        # Initialize states (at default) and update bit
        self.wheel_position = 0.0
        self.goal_position = 0.0
        self.last_joy_msgs = Joy()

    def _wheel_state(self, msg):
        self.last_joy_msgs = msg
        self.wheel_position = msg.axes[0]
        # self.get_logger().info(f'Wheel Position: {self.wheel_position}')

    def _get_env_state(self, request, response):
        response.wheel_position = self.wheel_position
        return response
    
    def _get_goal_position(self, _, response):
        response.wheel_position = self.goal_position
        return response
    
    def _get_joy_msg_buf(self, _, response):
        response.joy_buffer = self.last_joy_msgs
        return response
    
    def _goal_pose_callback(self, msg):
        # self.get_logger().info(f'Goal Position: {msg.position}, Torque: {msg.torque}')
        self.goal_position = msg.position

def main(args=None):
    rclpy.init(args=args)
    node = WheelBuffer()

    rclpy.spin(node)
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
