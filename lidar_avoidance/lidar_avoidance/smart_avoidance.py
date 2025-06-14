import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math

class ObstacleDecision(Node):
    def __init__(self):
        super().__init__('obstacle_decision')
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pub = self.create_publisher(String, '/lidar_status', 10)

    def scan_callback(self, msg):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        num_ranges = len(ranges)

        def angle_to_index(angle_deg):
            """Convert angle in degrees to index in ranges[]"""
            angle_rad = math.radians(angle_deg)
            index = int((angle_rad - angle_min) / angle_increment)
            return max(0, min(index, num_ranges - 1))

        def get_sector(start_deg, end_deg):
            """Get cleaned range values between angles"""
            start_idx = angle_to_index(start_deg)
            end_idx = angle_to_index(end_deg)

            if end_idx >= start_idx:
                sector = ranges[start_idx:end_idx]
            else:
                sector = ranges[start_idx:] + ranges[:end_idx]

            return [r for r in sector if 0.0 < r < float('inf')]

        # Define sectors (adjust angles if needed for your setup)
        front_sector = get_sector(-30, 30)
        left_sector = get_sector(60, 120)
        right_sector = get_sector(-120, -60)

        msg_out = String()

        if front_sector and min(front_sector) < 0.5:
            clear_left = self.count_clear(left_sector)
            clear_right = self.count_clear(right_sector)

            self.get_logger().info(f"Obstacle ahead. Left: {clear_left}, Right: {clear_right}")

            if clear_left > clear_right:
                msg_out.data = "LEFT"
            else:
                msg_out.data = "RIGHT"
        else:
            msg_out.data = "No obstacle. FORWARD."

        self.pub.publish(msg_out)

    def count_clear(self, sector):
        """Count how many values are reasonably clear (0.5m to 3.5m)"""
        return sum(1 for r in sector if 0.5 < r < 3.5)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDecision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
