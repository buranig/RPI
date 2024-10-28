#!/usr/bin/env python3
    
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from lar_msgs.msg import CarControlStamped
from sensor_msgs.msg import Joy
from bumper_msgs.srv import WheelPosition, JoyBuffer

# imports to read the potentiometer
import board
import time
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import RPi.GPIO as GPIO

# Initialize the I2C interface
i2c = busio.I2C(board.SCL, board.SDA)
# Create an ADS1115 object
ads = ADS.ADS1115(i2c)
# Define the analog input channel
channel = AnalogIn(ads, ADS.P0)

GPIO.setmode(GPIO.BCM)  # Set's GPIO pins to BCM GPIO numbering
INPUT_PIN = 4
GPIO.setup(INPUT_PIN, GPIO.IN)  # Set our input pin to be an input


class WheelBuffer(Node):
    def __init__(self):
        super().__init__('wheel_buffer')

        self.steering_ = self.create_subscription(Joy, '/joy', self._steering_state, qos_profile_sensor_data)

        self.publisher_ = self.create_publisher(CarControlStamped, "/sim/car/set/control", qos_profile_sensor_data)
        # Initialize states (at default) and update bit
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.wheel_position = 0.0
        self.last_joy_msgs = Joy()

    def _steering_state(self, msg):
        self.last_joy_msgs = msg
        self.wheel_position = msg.axes[0]

    def timer_callback(self):
        command = CarControlStamped()
        command.steering = self.wheel_position
        if (GPIO.input(INPUT_PIN) == True):
            command.throttle = float(channel.voltage/5.0*100)
        else:
            command:throttle = 0.0

        self.publisher_.publish(command)   

def main(args=None):
    rclpy.init(args=args)
    node = WheelBuffer()

    rclpy.spin(node)
    node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
