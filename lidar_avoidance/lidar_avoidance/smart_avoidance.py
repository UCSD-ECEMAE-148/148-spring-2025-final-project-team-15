import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math

class ObstacleDecision(Node):
    def __init__(self):
        super().__init__('obstacle_decision')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_decision_callback, 10)
        self.pub = self.create_publisher(String, '/lidar_status', 10)

    def scan_decision_callback(self, msg):
        ranges = msg.ranges
        total_pts = len(ranges)
#        angle_pos = 60  # how many points to take around the front
        def deg_to_index(deg):
            #convert degree to index based on a 360deg scan
#            rad = deg * 3.14159/180.0
            return int((deg+180)/360*total_pts)
        front_start = deg_to_index(-30)
        front_end = deg_to_index(30) #slicing out the indices for "front". "left" and "right" are defined wrt to front

        center_range = ranges[front_start:front_end] #center range = from start of front to end of front
        center_range = [r for r in center_range if r > 0.0 and r < float('inf')] #filter out inf and Nan

        msg_out = String()

        if center_range and min(center_range) < 0.5:
            # Obstacle detected — evaluate best direction
            right_sector = ranges[deg_to_index(30):deg_to_index(90)]
#            left_sector = ranges[deg_to_index(30):deg_to_index(90)]
            center_sector = ranges[front_start:front_end]
            left_sector = ranges[deg_to_index(-90):deg_to_index(-30)] #or left_sector

            #clean data
            left_sector = [r for r in left_sector if r > 0.0 and r < float('inf')]
            center_sector = [r for r in center_sector if r > 0.0 and r < float('inf')]
            right_sector = [r for r in right_sector if r > 0.0 and r < float('inf')]

#            print(right_sector)
#            print(center_sector)
#            print(left_sector)
            clear_left = self.count_clear(left_sector)
            clear_center = self.count_clear(center_sector)
            clear_right = self.count_clear(right_sector)
            self.get_logger().info(f"clear_left: {clear_left}")
            self.get_logger().info(f"clear_right: {clear_right}")
#            print("clear_left: {clear_left}")
#            print("clear_right: {clear_right}")
           
            #if clear_center < 10:
            if clear_center < clear_left or clear_center < clear_right:
                if clear_left > clear_right:
                    msg_out.data = "LEFT" #LEFT
                else:
                    msg_out.data = "RIGHT" #RIGHT
            else:
                msg_out.data = "FORWARD"
        else:
            msg_out.data = "No obstacle"

        self.pub.publish(msg_out)

    def count_clear(self, sector):
        return sum(1 for r in sector if r > 0.5 and r < 3.5)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDecision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
