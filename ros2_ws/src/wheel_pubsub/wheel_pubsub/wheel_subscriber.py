import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ros_g29_force_feedback.msg import ForceFeedback
from bumper_msgs.srv import WheelPosition

# from wheel_pubsub.device import Device
import evdev
from evdev import ecodes as e
import time

# from logidrivepy import LogitechController

class PID_controller():
    def __init__(self, initial_pos, target, kp = 0.9, ki = 0.0, kd = 0.02):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.target = target
        self.signal = 0
        self.accumulator = 0
        self.last_reading = initial_pos
        
        self.time_bkp = time.time()

        self.sample_rate = 0.1
    
    def compute_input(self, feedback_value):
        error = abs(self.target - feedback_value)

        self.accumulator += error

        self.signal = self.kp * error + self.ki * self.accumulator - self.kd * ( feedback_value - self.last_reading)/(time.time()-self.time_bkp)
        self.time_bkp = time.time()
    
    def update_state(self, feedback_value):
        self.last_reading = feedback_value

    def update_target(self, new_target):
        self.target = new_target

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        self.position_cli = self.create_client(WheelPosition, 'wheel_buffer')
        while not self.position_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.wheel_pos_req = WheelPosition.Request()
        wheel_position = self.wheel_pos_request()

        self.wheel_goal_cli = self.create_client(WheelPosition, 'wheel_goal')
        while not self.wheel_goal_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.wheel_goal_req = WheelPosition.Request()

        self.device_name = "/dev/input/event0"
        self.device = evdev.InputDevice(self.device_name)
        print(self.device.capabilities(verbose=True))
        if e.EV_FF in self.device.capabilities():
            print(f'Device: {self.device} is EV_FF event capable.')

        self.force_pub = self.create_publisher(ForceFeedback, '/ff_target',10)
        self.torque = 2.0
        self.dx = 0.08
        self.reverse = False
        self.wheel_goal = 0.0
        
        self.PID = PID_controller(self.wheel_pos_request(), self.wheel_goal)
        
        # perc  = 0
        # autocenter = str(int(perc / 100.0 * 65535))
        # self.device.write(e.EV_FF, e.FF_AUTOCENTER, int(autocenter))

        # gain = 100.0
        # value = 0xFFFF * gain / 100
        # self.device.write(e.EV_FF, e.FF_GAIN, int(value))
        
        # rumble = evdev.ff.Rumble(strong_magnitude=0x0000, weak_magnitude=0xffff)
        # effect_type = evdev.ff.EffectType(ff_rumble_effect=rumble)
        # duration_ms = 1000

        # effect = evdev.ff.Effect(
        #     e.FF_RUMBLE, -1, 0,
        #     evdev.ff.Trigger(0, 0),
        #     evdev.ff.Replay(duration_ms, 0),
        #     effect_type
        # )

        # repeat_count = 1
        # effect_id = self.device.upload_effect(effect)
        # self.device.write(e.EV_FF, effect_id, repeat_count)
        # time.sleep(duration_ms / 1000)
        # self.device.erase_effect(effect_id)

    def wheel_pos_request(self):
        self.wheel_pos_future = self.position_cli.call_async(self.wheel_pos_req)
        rclpy.spin_until_future_complete(self, self.wheel_pos_future)
        if not self.wheel_pos_future.done():
            print("Timeout")
        return self.wheel_pos_future.result().wheel_position
    
    def wheel_goal_request(self):
        self.wheel_goal_future = self.wheel_goal_cli.call_async(self.wheel_goal_req)
        rclpy.spin_until_future_complete(self, self.wheel_goal_future)
        if not self.wheel_goal_future.done():
            print("Timeout")
        return self.wheel_goal_future.result().wheel_position
    
    def calc_force(self):
        pass

    def write_gain(self, gain=100):
        print(f"write_gain {gain}")
        if self.supported_wheel() and self.supported_features()["ff_gain"] and e.EV_FF in self.get_device().capabilities():
            if gain is None:
                gain = 100
            elif gain > 100:
                gain = 100
            elif gain < 0:
                gain = 0
            self._gain = gain
            value = 0xFFFF * gain / 100
            self.get_device().write(e.EV_FF, e.FF_GAIN, int(value))

    def run(self):
        while rclpy.ok():
            # wheel_pos = -self.wheel_pos_request()
            # wheel_pos_goal = self.wheel_goal_request()
            msg = ForceFeedback()
            msg.header.stamp.sec = 0
            msg.header.stamp.nanosec = 0
            msg.header.frame_id = ''
            if self.wheel_goal < -0.5 and self.reverse==False:
                self.reverse=True
            elif self.wheel_goal > 0.5 and self.reverse==True:
                self.reverse = False

            msg.position = self.wheel_goal

            # self.PID.update_target(self.wheel_goal)
            # self.PID.compute_input(wheel_pos)
            # msg.torque = self.PID.signal
            # self.PID.update_state(wheel_pos)
            msg.torque = 0.0


            # self.get_logger().info(f'Position Difference: {self.wheel_goal-wheel_pos}, Torque: {self.PID.signal}')
            
            # self.get_logger().info(f'Wheel Position: {wheel_pos}, Wheel Goal: {self.wheel_goal}')

            if not self.reverse:
                self.wheel_goal = self.wheel_goal-self.dx
            else: 
                self.wheel_goal = self.wheel_goal+self.dx 

            # self.wheel_goal = 0.4
            

            self.force_pub.publish(msg)
            time.sleep(0.1)
            

        # Look at evdev documentation to see how to pass ff event


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    minimal_subscriber.run()
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
