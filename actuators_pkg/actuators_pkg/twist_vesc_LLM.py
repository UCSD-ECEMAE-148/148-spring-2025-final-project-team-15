import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from actuators_pkg.vesc_info_submodule.vesc_clientside import VESC_
import time
from std_msgs.msg import String

class VescTwist(Node):
    def __init__(self):
            super().__init__("twist_vesc_llm")
            self.vesc = VESC_()
            self.rpm_subscriber = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
            
            self.lidar_subscriber = self.create_subscription(String, '/lidar_status', self.lidar_callback, 10)
            self.last_msg_time = time.time()
            self.timeout_seconds = 2
            self.timeout_timer = self.create_timer(0.1, self.check_timeout)
            self.obstacle_detected = False
#       Default actuator values
            self.default_rpm_value = int(20000)
            self.default_steering_polarity = int(1) # if polarity is flipped, switch from 1 --> -1
            self.default_throttle_polarity = int(1) # if polarity is flipped, switch from 1 --> -1
            self.default_max_right_steering = 0.7
            self.default_straight_steering = 0.5
            self.default_max_left_steering = 0.3
            self.default_zero_throttle = -0.032
            self.default_max_throttle = 0.382
            self.default_min_throttle = 0.322
            # self.max_rpm = 3000
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('max_rpm', self.default_rpm_value),
                    ('steering_polarity', self.default_steering_polarity),
                    ('throttle_polarity', self.default_throttle_polarity),
                    ('max_right_steering', self.default_max_right_steering),
                    ('straight_steering', self.default_straight_steering),
                    ('max_left_steering', self.default_max_left_steering),
                    ('zero_throttle', self.default_zero_throttle),
                    ('max_throttle', self.default_max_throttle),
                    ('min_throttle', self.default_min_throttle)
                ])
            self.max_rpm = int(self.get_parameter('max_rpm').value)
            self.steering_polarity = int(self.get_parameter('steering_polarity').value)
            self.throttle_polarity = int(self.get_parameter('throttle_polarity').value)
            self.max_right_steering = self.get_parameter('max_right_steering').value
            self.straight_steering = self.get_parameter('straight_steering').value
            self.max_left_steering = self.get_parameter('max_left_steering').value
            self.zero_throttle = self.get_parameter('zero_throttle').value
            self.max_throttle = self.get_parameter('max_throttle').value
            self.min_throttle = self.get_parameter('min_throttle').value

            # Remappings
            # self.max_right_steering_angle = self.remap(self.max_right_steering)
            # self.steering_offset = self.remap(self.straight_steering) - self.default_straight_steering
            # self.max_left_steering_angle = self.remap(self.max_left_steering)
            # self.zero_rpm = int(self.zero_throttle * self.max_rpm)
            # self.max_rpm = int(self.max_throttle  * self.max_rpm)
            # self.min_rpm = int(self.min_throttle  * self.max_rpm)

            self.get_logger().info(
                f'\nmax_rpm: {self.max_rpm}'
                f'\nsteering_polarity: {self.steering_polarity}'
                f'\nthrottle_polarity: {self.throttle_polarity}'
                f'\nmax_right_steering: {self.max_right_steering}'
                f'\nstraight_steering: {self.straight_steering}'
                f'\nmax_left_steering: {self.max_left_steering}'
                #f'\nsteering_offset: {self.steering_offset}'
            )


    def callback(self, msg):
    # Clamp angular.z to valid range [-1.0, 1.0]
        clamped_z = max(min(msg.angular.z, 1.0), -1.0)

        # Compute steering angle by linear interpolation
        # angular.z = -1.0  → max left
        # angular.z =  0.0  → straight
        # angular.z = +1.0  → max right
        if self.obstacle_detected:
            self.get_logger().info("Ignoring cmd_vel value")
            return
        if clamped_z >= 0:
            steering_angle = (
                self.straight_steering +
                clamped_z * (self.max_right_steering - self.straight_steering)
            )
        else:
            steering_angle = (
                self.straight_steering +
                clamped_z * (self.straight_steering - self.max_left_steering)
            )

        # Clamp linear.x to [-1.0, 1.0] and scale to RPM
        clamped_speed = max(min(msg.linear.x, 1.0), -1.0)
        rpm = int(self.throttle_polarity * self.max_rpm * clamped_speed)

        # Send commands to VESC
        self.vesc.send_rpm(rpm)
        self.vesc.send_servo_angle(float(self.steering_polarity * steering_angle))
        self.last_msg_time = time.time()
    
    def lidar_callback(self, msg:String):
        if "obstacle detected" == msg.data.lower():
            if not self.obstacle_detected:
                self.get_logger().info("Lidar detected obstacle!")
            self.obstacle_detected = True
            self.vesc.send_rpm(0)
            self.vesc.send_servo_angle(float(self.steering_polarity * self.straight_steering))
            self.last_msg_time = time.time()
        else:
            self.obstacle_detected = False
 
    def check_timeout(self):
        if time.time() - self.last_msg_time > self.timeout_seconds:
            # No cmd_vel received recently — send stop + straight steering
            self.get_logger().info('No cmd_vel received recently, sending stop command.')
            self.vesc.send_rpm(0)
            self.vesc.send_servo_angle(float(self.steering_polarity * self.straight_steering))


    def remap(self, value):
        input_start = -1
        input_end = 1
        output_start = 0
        output_end = 1
        normalized_output = float(output_start + (value - input_start) * ((output_end - output_start) / (input_end - input_start)))
        return normalized_output

    def clamp(self, data, upper_bound, lower_bound=None):
        if lower_bound==None:
            lower_bound = -upper_bound # making lower bound symmetric about zero
        if data < lower_bound:
            data_c = lower_bound
        elif data > upper_bound:
            data_c = upper_bound
        else:
            data_c = data
        return data_c


def main(args=None):
    rclpy.init(args=args)
    try:
        vesc_twist = VescTwist()
        rclpy.spin(vesc_twist)
        vesc_twist.destroy_node()
        rclpy.shutdown()
    except:
        vesc_twist.get_logger().info(f'Could not connect to VESC, Shutting down...')
        vesc_twist.destroy_node()
        rclpy.shutdown()
        vesc_twist.get_logger().info(f'shut down successfully.')


if __name__ == '__main__':
    main()
