#!/usr/bin/env python3
"""Capture images from the robot camera for lane detection analysis."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

SAVE_DIR = '/home/arihant/ros2_ws/tools/captures'

class CameraCapture(Node):
    def __init__(self):
        super().__init__('camera_capture')
        self.bridge = CvBridge()
        self.latest_frame = None
        self.count = 0
        os.makedirs(SAVE_DIR, exist_ok=True)
        
        self.create_subscription(Image, '/camera/image_raw', self.cb, 1)
        self.get_logger().info(f"Listening to /camera/image_raw...")
        self.get_logger().info(f"Saving to {SAVE_DIR}")
        self.get_logger().info("Will capture 3 images, 2 seconds apart...")
        
        # Capture 3 images with a timer
        self.timer = self.create_timer(2.0, self.capture)

    def cb(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            pass

    def capture(self):
        if self.latest_frame is not None:
            self.count += 1
            fname = os.path.join(SAVE_DIR, f'camera_{self.count}.png')
            cv2.imwrite(fname, self.latest_frame)
            self.get_logger().info(f"[{self.count}/3] Saved: {fname}")
            if self.count >= 3:
                self.get_logger().info("Done! 3 images captured.")
                self.timer.cancel()
                raise SystemExit
        else:
            self.get_logger().warn("No frame received yet, waiting...")

def main():
    rclpy.init()
    node = CameraCapture()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
