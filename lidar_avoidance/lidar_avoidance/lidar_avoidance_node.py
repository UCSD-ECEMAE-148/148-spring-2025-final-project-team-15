import rclpy #ros2 library

from rclpy.node import Node #Node allows reading values from sensors
#sending motor commands
#some control

from sensor_msgs.msg import LaserScan #needed for LiDar

from geometry_msgs.msg import Twist #Twist for representing velocity of robot

class LidarAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_avoidance')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def scan_callback(self, msg):
        # Take the center 60-degree range (adjust based on your lidar)
        center_range = msg.ranges[len(msg.ranges)//2 - 30 : len(msg.ranges)//2 + 30]
        # Filter out 'inf' values
        center_range = [r for r in center_range if r > 0.0]

        twist = Twist()
        if center_range and min(center_range) < 0.5:
            twist.angular.z = 0.5  # Turn if obstacle ahead
        else:
            twist.linear.x = 0.2  # Move forward
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
