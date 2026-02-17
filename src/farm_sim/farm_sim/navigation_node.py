import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, LaserScan
from visualization_msgs.msg import Marker
import math
import csv
import os

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (bs, x, y, z, w) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class GPSConverter:
    """
    Converts between GPS (lat, lon) and local (x, y) coordinates (in meters).
    Assumes a flat Earth approximation over small distances.
    """
    def __init__(self, origin_lat, origin_lon):
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon

        # Earth's radius in meters
        R = 6378137.0

        # Meters per degree latitude is roughly constant
        self.meters_per_degree_lat = R * (math.pi / 180)

        # Meters per degree longitude varies with latitude (cos(lat))
        self.meters_per_degree_lon = R * (math.pi / 180) * math.cos(math.radians(origin_lat))

    def to_xy(self, lat, lon):
        """Converts GPS (lat, lon) to local (x, y) in meters.
        NOTE: Inverted signs to match Gazebo/Plugin coordinate frame quirk."""
        dx = -(lon - self.origin_lon) * self.meters_per_degree_lon
        dy = -(lat - self.origin_lat) * self.meters_per_degree_lat
        return dx, dy

    def to_latlon(self, x, y):
        """Converts local (x, y) in meters to GPS (lat, lon)."""
        lat = self.origin_lat - (y / self.meters_per_degree_lat)
        lon = self.origin_lon - (x / self.meters_per_degree_lon)
        return lat, lon

class SerpentineNavigator(Node):
    def __init__(self):
        super().__init__('serpentine_navigator')
        self.get_logger().info('Serpentine Navigator Node started.')

        # ROS Parameters matching user's config
        self.declare_parameter('farm_center_lat', 15.3478)
        self.declare_parameter('farm_center_lon', 75.1338)
        self.declare_parameter('farm_size_meters', 50.0) # User used 500 but simulation uses 50 for scale
        self.declare_parameter('grid_spacing_meters', 5.0) # User used 50
        self.declare_parameter('vehicle_speed', 0.5) 
        self.declare_parameter('angular_speed', 0.5) 
        self.declare_parameter('crop_csv_path', 'crop_coordinates.csv') # Default relative 

        self.farm_center_lat = self.get_parameter('farm_center_lat').get_parameter_value().double_value
        self.farm_center_lon = self.get_parameter('farm_center_lon').get_parameter_value().double_value
        self.farm_size_meters = self.get_parameter('farm_size_meters').get_parameter_value().double_value
        self.grid_spacing_meters = self.get_parameter('grid_spacing_meters').get_parameter_value().double_value
        self.vehicle_speed = self.get_parameter('vehicle_speed').get_parameter_value().double_value
        self.angular_speed = self.get_parameter('angular_speed').get_parameter_value().double_value

        # Publishers and Subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.marker_publisher = self.create_publisher(Marker, 'path_marker', 10)
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            '/demo/gps/fix',
            self.gps_callback,
            10
        )
        
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/demo/scan',
            self.scan_callback,
            10
        )

        # Vehicle state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.gps_received = False
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.obstacle_detected = False
        self.recovery_state = "NONE" # NONE, REVERSING, TURNING
        self.recovery_start_time = None

        # Navigation state
        self.target_gps_grid = []
        self.current_target_index = 0
        self.navigation_active = False
        self.path_points = []

        # Calculate farm bounds (User logic)
        approx_meters_per_degree_lat = 111000
        approx_meters_per_degree_lon = 111000 * math.cos(math.radians(self.farm_center_lat))

        lat_offset = (self.farm_size_meters / 2) / approx_meters_per_degree_lat
        lon_offset = (self.farm_size_meters / 2) / approx_meters_per_degree_lon

        self.lat_min_farm = self.farm_center_lat - lat_offset
        self.lon_min_farm = self.farm_center_lon - lon_offset
        self.lat_max_farm = self.farm_center_lat + lat_offset
        self.lon_max_farm = self.farm_center_lon + lon_offset

        # IMPORTANT: The GPSConverter origin must match the world origin coordinates
        # In hubli_farm.world, the origin (0,0,0) is at Lat: 15.3478, Lon: 75.1338
        self.gps_converter = GPSConverter(origin_lat=self.farm_center_lat, origin_lon=self.farm_center_lon)

        # Generate the GPS grid behavior is now triggered by first GPS fix in gps_callback
        # self.generate_gps_grid()

        # Navigation timer
        self.timer = self.create_timer(0.1, self.navigate_serpentine) 

    def scan_callback(self, msg):
        # Check for obstacles in front (-30 to +30 degrees)
        # Laser ranges: index 0 is -pi (back), index mid is 0 (front)
        if not msg.ranges:
            return
            
        num_readings = len(msg.ranges)
        mid_index = num_readings // 2
        window = num_readings // 12 # ~30 degrees
        
        # Ranges can contain inf/nan, filter them
        front_ranges = [r for r in msg.ranges[mid_index - window : mid_index + window] 
                        if not math.isinf(r) and not math.isnan(r)]
        
        min_front = min(front_ranges) if front_ranges else 10.0
        
        if min_front < 1.0: # Obstacle < 1m
            self.get_logger().info(f'Scan: Min Front Dist = {min_front:.2f} (OBSTACLE)', throttle_duration_sec=1.0)
            self.obstacle_detected = True
        else:
            self.get_logger().info(f'Scan: Min Front Dist = {min_front:.2f} (Clear)', throttle_duration_sec=1.0)
            self.obstacle_detected = False 

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
        if not self.gps_received:
            self.get_logger().info(f'First GPS Fix: Lat: {msg.latitude}, Lon: {msg.longitude}.')
            self.gps_received = True
            
            # Generate path once we have confirmation of GPS system working (or just start it)
            # The origin remains fixed at the Farm Center (World Origin)
            self.target_gps_grid = []
            self.current_target_index = 0
            self.generate_smart_path()

        self.current_x, self.current_y = self.gps_converter.to_xy(msg.latitude, msg.longitude)
        
        # DEBUG: Print status periodically
        # self.get_logger().info(
        #     f"GPS: ({msg.latitude:.6f}, {msg.longitude:.6f}) -> Local: ({self.current_x:.2f}, {self.current_y:.2f})", 
        #     throttle_duration_sec=2.0
        # )

    def generate_smart_path(self):
        csv_path = self.get_parameter('crop_csv_path').get_parameter_value().string_value
        if not os.path.exists(csv_path):
            self.get_logger().error(f"Crop CSV not found at {csv_path}. Falling back to default grid.")
            self.generate_gps_grid()
            return

        self.get_logger().info(f"Reading crops for smart path from {csv_path}...")
        xs, ys = [], []
        try:
            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    xs.append(float(row['x']))
                    ys.append(float(row['y']))
        except Exception as e:
            self.get_logger().error(f"Failed to read CSV: {e}")
            return

        if not xs:
            return

        # Buffer limits
        min_x, max_x = min(xs) - 2.0, max(xs) + 2.0
        min_y, max_y = min(ys), max(ys)
        
        self.get_logger().info(f"Field  Bounds: X[{min_x:.1f}, {max_x:.1f}], Y[{min_y:.1f}, {max_y:.1f}]")

        # Logic: Start from Top-Right (Max X, Top Gap).
        # Top Crop Y is ~5.0. First gap is ~4.0.
        # We start iterating Y from (max_y - 1.0) downwards.
        
        current_y = 4.0 # Hardcoded alignment based on known field structure (gaps at 4, 2, 0...)
        # Ideally: current_y = round(max_y) - 1.0
        
        self.target_gps_grid = []
        
        while current_y >= -4.0: # Bottom gap is -4.0
            # Pass 1: Right to Left (Positive X to Negative X)
            # Add Start (Right)
            lat, lon = self.gps_converter.to_latlon(max_x, current_y)
            self.target_gps_grid.append((lat, lon))
            
            # Add End (Left)
            lat, lon = self.gps_converter.to_latlon(min_x, current_y)
            self.target_gps_grid.append((lat, lon))
            
            current_y -= 2.0 # Move to next gap
            
            if current_y >= -4.0:
                # Pass 2: Left to Right (Negative X to Positive X)
                # Add Start (Left)
                lat, lon = self.gps_converter.to_latlon(min_x, current_y)
                self.target_gps_grid.append((lat, lon))
                
                # Add End (Right)
                lat, lon = self.gps_converter.to_latlon(max_x, current_y)
                self.target_gps_grid.append((lat, lon))
                
                current_y -= 2.0 

        self.get_logger().info(f'Generated {len(self.target_gps_grid)} smart waypoints (Top-Right to Bottom-Left).')
        
        # Export waypoints to CSV
        csv_path = 'robot_waypoints.csv'
        with open(csv_path, 'w') as f:
            f.write("lat,lon,x,y\n")
            for lat, lon in self.target_gps_grid:
                x, y = self.gps_converter.to_xy(lat, lon)
                f.write(f"{lat},{lon},{x:.2f},{y:.2f}\n")
        self.get_logger().info(f'Exported waypoints to {csv_path}')

        self.navigation_active = True

    def navigate_serpentine(self):
        # Debug log to verify timer is firing
        # self.get_logger().info('Navigation loop running...', throttle_duration_sec=5.0)

        if not self.navigation_active:
            return

        # Obstacle Recovery Logic
        now = self.get_clock().now()
        
        if self.recovery_state == "NONE":
            if self.obstacle_detected:
                self.get_logger().warn("Obstacle detected! Starting Recovery: REVERSING")
                self.recovery_state = "REVERSING"
                self.recovery_start_time = now
                self.stop_vehicle()
                return # Yield control

        elif self.recovery_state == "REVERSING":
            elapsed = (now - self.recovery_start_time).nanoseconds / 1e9
            if elapsed < 2.0:
                twist_msg = Twist()
                twist_msg.linear.x = -0.2 # Back up
                self.cmd_vel_publisher.publish(twist_msg)
                return
            else:
                self.get_logger().info("Recovery: Switching to TURNING")
                self.recovery_state = "TURNING"
                self.recovery_start_time = now
                return

        elif self.recovery_state == "TURNING":
            elapsed = (now - self.recovery_start_time).nanoseconds / 1e9
            # Turn for at least 1s, or until clear (with max timeout of 3s)
            if elapsed < 1.0 or (self.obstacle_detected and elapsed < 3.0):
                twist_msg = Twist()
                twist_msg.angular.z = 0.8 # Turn left
                self.cmd_vel_publisher.publish(twist_msg)
                return
            else:
                self.get_logger().info("Recovery: Path Clear (or timeout). Resuming.")
                self.recovery_state = "NONE"
                # Fall through to normal navigation

        if not self.gps_received:
            self.get_logger().info('Waiting for GPS fix...', throttle_duration_sec=2.0)
            return

        if self.current_target_index >= len(self.target_gps_grid):
            self.get_logger().info('Serpentine path complete!')
            self.navigation_active = False
            self.stop_vehicle()
            return

        target_lat, target_lon = self.target_gps_grid[self.current_target_index]
        target_x, target_y = self.gps_converter.to_xy(target_lat, target_lon)
        
        distance_to_target = math.dist((self.current_x, self.current_y), (target_x, target_y))

        twist_msg = Twist()

        if distance_to_target > 0.5: # Tolerance
            angle_to_target = math.atan2(target_y - self.current_y, target_x - self.current_x)
            
            # Normalize yaw to -pi to pi
            angle_diff = angle_to_target - self.current_yaw
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            if abs(angle_diff) > 0.2: # Turn if not facing target
                twist_msg.angular.z = self.angular_speed * (1 if angle_diff > 0 else -1)
                twist_msg.linear.x = 0.0
                state = "TURNING"
            else:
                twist_msg.linear.x = self.vehicle_speed
                twist_msg.angular.z = 0.0 # Drive straight
            state = "MOVING"
            
            self.get_logger().info(
                f'[{state}] Cur:({self.current_x:.2f}, {self.current_y:.2f}) '
                f'Tgt:({target_x:.2f}, {target_y:.2f}) '
                f'Dist: {distance_to_target:.2f}m, AngDiff: {angle_diff:.2f}',
                throttle_duration_sec=0.5
            )
        else:
            self.get_logger().info(f'Reached waypoint {self.current_target_index + 1}/{len(self.target_gps_grid)}')
            self.current_target_index += 1
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0

        if self.gps_received:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "path"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.1 # Line width
            marker.color.a = 1.0
            marker.color.b = 1.0 # Blue
            marker.header.frame_id = "odom" 
            # Note: "odom" frame assumes odom is published by plugins. 
            # If not, "dummy" or "base_link" might be needed but "odom" is standard for fixed frame path.

            p = Point()
            p.x = self.current_x
            p.y = self.current_y
            p.z = 0.1
            self.path_points.append(p)
            marker.points = self.path_points
            self.marker_publisher.publish(marker)

        self.cmd_vel_publisher.publish(twist_msg)

    def stop_vehicle(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    navigator = SerpentineNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
