import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32
import cv_bridge
import cv2
import numpy as np

class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')
        
        # ROS parameters for dynamic tuning
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('hough_threshold', 50)
        self.declare_parameter('hough_min_line_len', 100)
        self.declare_parameter('hough_max_line_gap', 50)
        self.declare_parameter('curvature_threshold', 0.2)
        
        # Create a CV bridge to convert ROS Images to OpenCV Images
        self.bridge = cv_bridge.CvBridge()

        # Input and output ROS topics
        self.sub_image = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            1)  # small QoS queue to prevent processing backups/deadlocks
            
        self.pub_polygon = self.create_publisher(
            PolygonStamped, 
            '/lane_boundaries', 
            10)
            
        # Optional: Publish an annotated image to visualize debugging
        self.pub_annotated_image = self.create_publisher(
            Image,
            '/lane_boundaries_image',
            10)
            
        self.get_logger().info('Lane Detector Node initialized')
        
        # State variables for smoothing/filtering (to handle noisy sensors)
        self.left_line_history = []
        self.right_line_history = []
        self.history_length = 5

    def get_params(self):
        return {
            'canny_low': self.get_parameter('canny_low').value,
            'canny_high': self.get_parameter('canny_high').value,
            'hough_threshold': self.get_parameter('hough_threshold').value,
            'hough_min_line_len': self.get_parameter('hough_min_line_len').value,
            'hough_max_line_gap': self.get_parameter('hough_max_line_gap').value,
            'curvature_threshold': self.get_parameter('curvature_threshold').value
        }

    def region_of_interest(self, img, vertices):
        mask = np.zeros_like(img)
        match_mask_color = 255
        cv2.fillPoly(mask, vertices, match_mask_color)
        masked_image = cv2.bitwise_and(img, mask)
        return masked_image

    def filter_lines(self, lines, curvature_threshold):
        left_lines = []
        right_lines = []
        if lines is None:
            return left_lines, right_lines

        for line in lines:
            for x1, y1, x2, y2 in line:
                if x1 == x2:
                    continue  # Ignore vertical lines
                slope = (y2 - y1) / (x2 - x1)
                
                # Filter out almost horizontal lines and extreme noise using curvature_threshold
                if abs(slope) < curvature_threshold:
                    continue
                    
                if slope < 0:
                    left_lines.append(line)
                else:
                    right_lines.append(line)
        return left_lines, right_lines

    def average_lines(self, image, lines, is_left):
        if len(lines) == 0:
            return None
        
        x_coords = []
        y_coords = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                x_coords.extend([x1, x2])
                y_coords.extend([y1, y2])
                
        # Fit a line (1st degree polynomial) to the points
        poly = np.polyfit(y_coords, x_coords, 1)
        
        y1 = image.shape[0]        # Bottom of image
        y2 = int(image.shape[0]*0.6) # Approx horizon line
        
        x1 = int(np.polyval(poly, y1))
        x2 = int(np.polyval(poly, y2))
        
        return [x1, y1, x2, y2]

    def smooth_lines(self, new_line, history):
        if new_line is None:
            if len(history) > 0:
                return history[-1]
            return None

        history.append(new_line)
        if len(history) > self.history_length:
            history.pop(0)
            
        avg_line = np.mean(history, axis=0, dtype=np.int32)
        return avg_line.tolist()

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (BGR8)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        params = self.get_params()
        
        # 1. Grayscale & Blur
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # 2. Canny Edge Detection
        edges = cv2.Canny(blur, params['canny_low'], params['canny_high'])
        
        # 3. Region of Interest Masking (bottom trapezoid)
        height, width = edges.shape
        roi_vertices = [
            (0, height),
            (width / 2 - 50, height * 0.6),
            (width / 2 + 50, height * 0.6),
            (width, height)
        ]
        masked_edges = self.region_of_interest(edges, np.array([roi_vertices], np.int32))
        
        # 4. Hough Transform for line detection
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi/180,
            threshold=params['hough_threshold'],
            minLineLength=params['hough_min_line_len'],
            maxLineGap=params['hough_max_line_gap']
        )
        
        # 5. Filter & Classify Lines by Curvature/Slope
        left_lines, right_lines = self.filter_lines(lines, params['curvature_threshold'])
        
        # 6. Average and Smooth Lines
        left_avg = self.average_lines(cv_image, left_lines, is_left=True)
        right_avg = self.average_lines(cv_image, right_lines, is_left=False)
        
        smoothed_left = self.smooth_lines(left_avg, self.left_line_history)
        smoothed_right = self.smooth_lines(right_avg, self.right_line_history)
        
        # 7. Publish geometry_msgs/PolygonStamped (ensuring exact coordinate match to fix mismatches)
        polygon_msg = PolygonStamped()
        polygon_msg.header = msg.header
        
        annotated_img = cv_image.copy()
        
        if smoothed_left is not None:
            # Publish left lane boundary
            p1 = Point32(x=float(smoothed_left[0]), y=float(smoothed_left[1]), z=0.0)
            p2 = Point32(x=float(smoothed_left[2]), y=float(smoothed_left[3]), z=0.0)
            polygon_msg.polygon.points.extend([p1, p2])
            cv2.line(annotated_img, (smoothed_left[0], smoothed_left[1]), (smoothed_left[2], smoothed_left[3]), (0, 0, 255), 5)
            
        if smoothed_right is not None:
            # Publish right lane boundary 
            p3 = Point32(x=float(smoothed_right[2]), y=float(smoothed_right[3]), z=0.0)
            p4 = Point32(x=float(smoothed_right[0]), y=float(smoothed_right[1]), z=0.0)
            polygon_msg.polygon.points.extend([p3, p4])
            cv2.line(annotated_img, (smoothed_right[0], smoothed_right[1]), (smoothed_right[2], smoothed_right[3]), (255, 0, 0), 5)
            
        self.pub_polygon.publish(polygon_msg)

        # 8. Optional: Publish the annotated visualization
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_img, "bgr8")
            annotated_msg.header = msg.header
            self.pub_annotated_image.publish(annotated_msg)
        except Exception as e:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
