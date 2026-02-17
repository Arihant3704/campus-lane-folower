import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge
import numpy as np

class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        self.publisher_ = self.create_publisher(Bool, '/red_detected', 10)
        self.subscription = self.create_subscription(
            Image,
            '/demo/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Define range for red color and create a mask
        # These values might need tuning depending on the exact red in Gazebo
        lower_red = np.array([0, 0, 100])
        upper_red = np.array([50, 50, 255])
        mask = cv2.inRange(cv_image, lower_red, upper_red)

        # Check if any red is detected
        red_detected = np.any(mask)
        
        # Publish the detection result
        detection_msg = Bool()
        detection_msg.data = bool(red_detected)
        self.publisher_.publish(detection_msg)
        
        if red_detected:
            self.get_logger().info("Red object detected!")

def main(args=None):
    rclpy.init(args=args)
    color_detector = ColorDetectorNode()
    rclpy.spin(color_detector)
    color_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
