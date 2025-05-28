import rclpy #ros2 library

from rclpy.node import Node #Node allows reading values from sensors
#sending motor commands
#some control

from sensor_msgs.msg import LaserScan #needed for LiDar. receivign readings from lidar

from geometry_msgs.msg import Twist #Twist for representing velocity of robot

class LidarAvoidance(Node): #creating LidarAvoidance that inherits from Node
    def __init__(self): #constructor 
        super().__init__('lidar_avoidance') #node name: lidar_avoidance
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
		#Message type = LaserScan (dor distance measuring)
		# topic subscribing to /scan --> /scan topic receives LaserScan messages
		# calls scan_callback when new message arrives
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
		# send Twist messages (gives robot instruction on how to move in terms of speed/velocity)  to /cmd_vel topic

    def scan_callback(self, msg):  #call when new lidar data is received. detects obstacles in front
			# msg = input from lidar 
        # take the center 60-degree range (adjust based on your lidar)
        center_range = msg.ranges[len(msg.ranges)//2 - 30 : len(msg.ranges)//2 + 30]
        # filter out 'inf' values
        center_range = [r for r in center_range if r > 0.0]

        twist = Twist()
        if center_range and min(center_range) < 0.5:
            twist.angular.z = 0.5  # turn if obstacle ahead
        else:
            twist.linear.x = 0.2  # move forward
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
