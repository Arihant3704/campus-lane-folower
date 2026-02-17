import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import math

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/waypoint_target', 10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/demo/odom',
            self.odom_callback,
            10)

        # Waypoints that trace the center of the racetrack
        self.waypoints = [
            (10.0, 0.0),
            (20.0, 10.0),
            (10.0, 20.0),
            (-10.0, 20.0),
            (-20.0, 10.0),
            (-10.0, 0.0),
            (0.0, 0.0) # Back to start
        ]
        self.current_waypoint_index = 0
        self.current_pose = None
        self.distance_threshold = 1.5  # How close the car needs to be to a waypoint to consider it "reached"

        # Publish the first waypoint immediately
        self.publish_current_waypoint()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.check_waypoint_reached()

    def check_waypoint_reached(self):
        if self.current_pose is None:
            return

        target_x, target_y = self.waypoints[self.current_waypoint_index]
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        distance = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)

        if distance < self.distance_threshold:
            self.get_logger().info(f"Waypoint {self.current_waypoint_index} reached!")
            # Advance to the next waypoint, and loop back to the start if we're at the end
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
            self.publish_current_waypoint()

    def publish_current_waypoint(self):
        x, y = self.waypoints[self.current_waypoint_index]
        
        waypoint_msg = PoseStamped()
        waypoint_msg.header.frame_id = "odom"
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.pose.position.x = x
        waypoint_msg.pose.position.y = y
        waypoint_msg.pose.orientation.w = 1.0
        
        self.publisher_.publish(waypoint_msg)
        self.get_logger().info(f"Publishing new target waypoint ({self.current_waypoint_index}): ({x}, {y})")


def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
