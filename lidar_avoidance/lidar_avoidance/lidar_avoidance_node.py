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
        self.status_pub = self.create_publisher(String, '/lidar_status', 10)
		# send Twist messages (gives robot instruction on how to move in terms of speed/velocity)  to /cmd_vel topic

    def scan_callback(self, msg):  #call when new lidar data is received. detects obstacles in front
	# msg = input from lidar 
        import math
        view_degree = 180
        # print("LiDAR data received:", msg.ranges[:5])
        angle_pos = 60
        # take the center 180-degree (90 to the left of the center line and 90 to the right of the center line) range (adjust based on your lidar)
       	center_range = msg.ranges[len(msg.ranges)//2 - angle_pos : len(msg.ranges)//2 + angle_pos]
        # filter out 'inf' values
        # center_range = [r for r in center_range if r > 0.0]
        center_range = [r for r in center_range if r > 0.0 and r != float('inf') and not math.isnan(r)]
        print("Filtered LiDAR Data:", center_range)
        twist = Twist()
        status_msg = String()
        if center_range and min(center_range) < 0.5: #turning when the lidar is 0.5m away from the obstacle
            print("Obstacle detected! Turning to avoid.")
            twist.angular.z = 0.5  # turn if obstacle ahead. 0.5 rad/s ~= 30deg
            status_msg.data = "Obstacle detected"
        else:
            twist.linear.x = 0.2  # move forward
            status_msg.data = "No obstacle"
        self.pub.publish(twist)
        self.status_pub.publish(status_msg)
        #self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
