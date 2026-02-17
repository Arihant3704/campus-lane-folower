import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
from tf_transformations import euler_from_quaternion
from enum import Enum

# --- Behavior Tree Framework ---
class BTStatus(Enum):
    SUCCESS = 1
    FAILURE = 2
    RUNNING = 3

class BTNode:
    def __init__(self, node):
        self.node = node
    def tick(self):
        raise NotImplementedError

class Selector(BTNode):
    def __init__(self, node, children=[]):
        super().__init__(node)
        self.children = children
    def tick(self):
        for child in self.children:
            status = child.tick()
            if status != BTStatus.FAILURE:
                return status
        return BTStatus.FAILURE

class Sequence(BTNode):
    def __init__(self, node, children=[]):
        super().__init__(node)
        self.children = children
    def tick(self):
        for child in self.children:
            status = child.tick()
            if status != BTStatus.SUCCESS:
                return status
        return BTStatus.SUCCESS

# --- Behavior Tree Nodes for the Car ---
class IsRedDetected(BTNode):
    def tick(self):
        return BTStatus.SUCCESS if self.node.red_detected else BTStatus.FAILURE

class Stop(BTNode):
    def tick(self):
        self.node.get_logger().info("BT: Stop action")
        twist = Twist()
        self.node.publisher_.publish(twist)
        return BTStatus.SUCCESS

class IsObstacleNear(BTNode):
    def tick(self):
        return BTStatus.SUCCESS if self.node.min_laser_dist < self.node.obstacle_distance_threshold else BTStatus.FAILURE

class AvoidObstacle(BTNode):
    def tick(self):
        self.node.get_logger().info("BT: Avoid Obstacle action")
        
        # Attractive force (towards waypoint)
        if self.node.target_pose is None or self.node.current_pose is None:
            return BTStatus.FAILURE
        
        target_x = self.node.target_pose.pose.position.x
        target_y = self.node.target_pose.pose.position.y
        current_x = self.node.current_pose.position.x
        current_y = self.node.current_pose.position.y
        
        attractive_force_x = self.node.attractive_gain * (target_x - current_x)
        attractive_force_y = self.node.attractive_gain * (target_y - current_y)

        # Repulsive force (from obstacles)
        repulsive_force_x = 0.0
        repulsive_force_y = 0.0
        
        for i, r in enumerate(self.node.laser_ranges):
            if r < self.node.obstacle_distance_threshold:
                angle = self.node.laser_angle_min + i * self.node.laser_angle_increment
                # Scale repulsive force by inverse of distance
                force_mag = self.node.repulsive_gain * (1.0/r - 1.0/self.node.obstacle_distance_threshold)
                repulsive_force_x -= force_mag * math.cos(angle)
                repulsive_force_y -= force_mag * math.sin(angle)

        # Total force
        total_force_x = attractive_force_x + repulsive_force_x
        total_force_y = attractive_force_y + repulsive_force_y

        # Convert force to Twist command
        _, _, current_yaw = euler_from_quaternion([
            self.node.current_pose.orientation.x,
            self.node.current_pose.orientation.y,
            self.node.current_pose.orientation.z,
            self.node.current_pose.orientation.w])
            
        angle_to_force = math.atan2(total_force_y, total_force_x)
        heading_error = angle_to_force - current_yaw
        if heading_error > math.pi: heading_error -= 2 * math.pi
        elif heading_error < -math.pi: heading_error += 2 * math.pi

        twist = Twist()
        twist.angular.z = self.node.angular_speed_max * (heading_error / math.pi)
        # Slow down if there's a large heading error
        twist.linear.x = self.node.linear_speed * (1.0 - abs(heading_error) / math.pi)

        self.node.publisher_.publish(twist)
        return BTStatus.RUNNING

class NavigateToWaypoint(BTNode):
    def tick(self):
        if self.node.target_pose is None or self.node.current_pose is None:
            return BTStatus.FAILURE
            
        self.node.get_logger().info("BT: Navigate to Waypoint action")
        target_x = self.node.target_pose.pose.position.x
        target_y = self.node.target_pose.pose.position.y
        current_x = self.node.current_pose.position.x
        current_y = self.node.current_pose.position.y
        orientation_q = self.node.current_pose.orientation
        _, _, current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        heading_error = angle_to_target - current_yaw
        if heading_error > math.pi: heading_error -= 2 * math.pi
        elif heading_error < -math.pi: heading_error += 2 * math.pi
        
        distance_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        
        twist = Twist()
        if distance_to_target > 0.2:
            twist.angular.z = self.node.angular_speed_max * (heading_error / math.pi)
            twist.linear.x = self.node.linear_speed * min(1.0, distance_to_target)
            self.node.publisher_.publish(twist)
            return BTStatus.RUNNING
        else:
            self.node.get_logger().info("Waypoint reached!")
            self.node.target_pose = None
            twist = Twist()
            self.node.publisher_.publish(twist)
            return BTStatus.SUCCESS

# --- Main ROS 2 Node ---
class CarControllerNode(Node):
    def __init__(self):
        super().__init__('car_simulation_node')
        self.publisher_ = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        self.create_subscription(LaserScan, '/demo/scan', self.laser_callback, 10)
        self.create_subscription(PoseStamped, '/waypoint_target', self.waypoint_callback, 10)
        self.create_subscription(Odometry, '/demo/odom', self.odom_callback, 10)
        self.create_subscription(Bool, '/red_detected', self.red_detected_callback, 10)

        # State variables
        self.target_pose = None
        self.current_pose = None
        self.red_detected = False
        self.min_laser_dist = float('inf')
        self.laser_ranges = []
        self.laser_angle_min = 0.0
        self.laser_angle_increment = 0.0

        # Parameters
        self.linear_speed = 0.3
        self.angular_speed_max = 1.0
        self.obstacle_distance_threshold = 1.2
        self.attractive_gain = 0.1
        self.repulsive_gain = 0.3

        # Build the Behavior Tree
        self.root = Selector(self, children=[
            Sequence(self, children=[IsRedDetected(self), Stop(self)]),
            Sequence(self, children=[IsObstacleNear(self), AvoidObstacle(self)]),
            NavigateToWaypoint(self)
        ])

        # Create a timer to tick the BT
        self.create_timer(0.1, self.update_loop)

    def waypoint_callback(self, msg):
        self.target_pose = msg
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
    def red_detected_callback(self, msg):
        self.red_detected = msg.data
    def laser_callback(self, msg):
        self.laser_ranges = msg.ranges
        self.laser_angle_min = msg.angle_min
        self.laser_angle_increment = msg.angle_increment
        self.min_laser_dist = min(filter(lambda x: x > msg.range_min, msg.ranges), default=float('inf'))

    def update_loop(self):
        self.root.tick()

def main(args=None):
    rclpy.init(args=args)
    car_controller = CarControllerNode()
    rclpy.spin(car_controller)
    car_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
