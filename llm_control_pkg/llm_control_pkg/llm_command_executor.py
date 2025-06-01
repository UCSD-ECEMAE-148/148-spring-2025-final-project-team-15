import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

#we need twist commands for velocity and rotation
#type twist

ACTUATOR_TOPIC_NAME = '/cmd_vel'

class LLMexecutor(Node):
    def __init__(self):
        super().__init__('command_executor')
        self.subscription = self.create_subscription(String,'voice_command',self.listener_callback,10) #subscribes to voice command
        self.twist_publisher = self.create_publisher(Twist, ACTUATOR_TOPIC_NAME,10) #publishes to /cmd_vel
        self.twist_cmd = Twist()

    #default actuator vals
        self.declare_parameters(
                namespace='',
                parameters=[
                ('zero_throttle',0.0),
                ('max_throttle', 0.2),
                    ('min_throttle', 0.1),
                ('max_right_steering', 1.0),
                ('max_left_steering', -1.0)
        ])

        self.zero_throttle = self.get_parameter('zero_throttle').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.min_throttle = self.get_parameter('min_throttle').value
        self.max_right_steering = self.get_parameter('max_right_steering').value
        self.max_left_steering = self.get_parameter('max_left_steering').value

    def listener_callback(self,msg):
        self.get_logger().info(f'Executing: {msg.data}')
        command = msg.data.lower().strip()
        direction, speed, duration, angle = command.split(',')
        self.get_logger().info(f'direction:{direction}, speed:{speed}, duration:{duration}, angle:{angle}')
        twist = Twist()

        twist.linear.x = float(direction)*float(speed)
        self.get_logger().info(f'Linear updated to: {twist.linear.x}')
        twist.angular.z = float(angle)
        start = time.time()
        while time.time() - start < int(duration):
            self.twist_publisher.publish(twist)
            time.sleep(0.1)
        self.stop_motion()

    def stop_motion(self):
        stop_twist = Twist()
        stop_twist.linear.x = self.zero_throttle
        stop_twist.angular.z = 0.0
        self.twist_publisher.publish(stop_twist)
        self.get_logger().info("Stopped motion")

    #handle stuff before publishing
    def controller(self, data):
        self.get_logger().info(f"Controller received: COMPLETE LOGIC HERE")

def main(args=None):
    rclpy.init(args=args)
    executor = LLMexecutor()
    rclpy.spin(executor)
    executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("Hello, stuff is happening")
    main()
