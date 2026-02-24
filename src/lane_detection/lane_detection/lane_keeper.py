import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class LaneFollowerNode(Node):
    def __init__(self):
        super().__init__('lane_follower_node')
        
        # 1. Tuning Parameters
        self.declare_parameter('speed', 0.5)
        self.declare_parameter('kp', 0.015)
        
        self.bridge = CvBridge()
        self.enabled = False
        self.image_received = False
        
        # 2. Subscriptions
        self.create_subscription(Image, '/camera/image_raw', self.process_image, 10)
        self.create_subscription(Bool, '/lane_following_enabled', self.enable_callback, 10)
        
        # 3. Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vis_pub = self.create_publisher(Image, '/lane_debug', 10)
        
        self.get_logger().info("--- LANE FOLLOWER LOGIC READY ---")
        self.timer = self.create_timer(2.0, self.check_status)

    def check_status(self):
        if not self.image_received:
            self.get_logger().warn("Waiting for camera images on /camera/image_raw...")
        if not self.enabled:
            self.get_logger().info("Autonomous mode: IDLE (Wait for GUI button click)")

    def enable_callback(self, msg):
        self.enabled = msg.data
        self.get_logger().info(f"SWITCH: Autonomous Mode {'ON' if self.enabled else 'OFF'}")

    def process_image(self, msg):
        self.image_received = True
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            return

        h, w, _ = frame.shape
        img_center = w / 2

        # =============================================
        # A. HSV COLOR FILTERING
        # =============================================
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # White (Center dotted line)
        mask_w = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 40, 255]))
        
        # Yellow (Edge boundaries)
        mask_y = cv2.inRange(hsv, np.array([10, 50, 50]), np.array([40, 255, 255]))
        
        # Clean noise
        kernel = np.ones((3, 3), np.uint8)
        mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_OPEN, kernel)
        mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN, kernel)

        # =============================================
        # B. ROI: Bottom 50% of image (Task #3)
        # =============================================
        roi_top = int(h * 0.5)
        roi_mask_w = mask_w[roi_top:, :]
        roi_mask_y = mask_y[roi_top:, :]

        # =============================================
        # C. FIND LANE BOUNDARIES (split left/right)
        # =============================================
        left_yellow = roi_mask_y[:, :int(w/2)]
        right_yellow = roi_mask_y[:, int(w/2):]
        
        left_pix = np.where(left_yellow > 0)
        right_pix = np.where(right_yellow > 0)
        
        l_found = len(left_pix[1]) > 20
        r_found = len(right_pix[1]) > 20
        
        # White center line detection in ROI
        white_pix = np.where(roi_mask_w > 0)
        w_found = len(white_pix[1]) > 20
        
        # =============================================
        # D. COMPUTE TARGET CENTER
        # =============================================
        detection_mode = "LOST"
        
        if l_found and r_found:
            l_edge = np.mean(left_pix[1])
            r_edge = np.mean(right_pix[1]) + w/2
            target_center = (l_edge + r_edge) / 2
            found = True
            detection_mode = "BOTH EDGES"
        elif l_found and w_found:
            l_edge = np.mean(left_pix[1])
            w_center = np.mean(white_pix[1])
            target_center = (l_edge + w_center) / 2
            found = True
            detection_mode = "LEFT + WHITE"
        elif r_found and w_found:
            r_edge = np.mean(right_pix[1]) + w/2
            w_center = np.mean(white_pix[1])
            target_center = (w_center + r_edge) / 2
            found = True
            detection_mode = "RIGHT + WHITE"
        elif w_found:
            target_center = np.mean(white_pix[1])
            found = True
            detection_mode = "WHITE ONLY"
        elif l_found:
            l_edge = np.mean(left_pix[1])
            target_center = l_edge + 180
            found = True
            detection_mode = "LEFT ONLY"
        elif r_found:
            r_edge = np.mean(right_pix[1]) + w/2
            target_center = r_edge - 180
            found = True
            detection_mode = "RIGHT ONLY"
        else:
            target_center = img_center
            found = False

        # =============================================
        # E. CONTROL (Task #2 & #4: speed + steering)
        # =============================================
        error = img_center - target_center
        kp = self.get_parameter('kp').value
        steering = error * kp
        steering = max(-1.5, min(1.5, steering))  # Clamp

        # Task #2: Maintain FULL speed during turns (no slowdown)
        # Task #4: Increase speed on straights, maintain on curves
        base_speed = self.get_parameter('speed').value
        abs_steer = abs(steering)
        
        if abs_steer < 0.1:
            # Straight road - full speed
            speed = base_speed
            drive_mode = "STRAIGHT"
        elif abs_steer < 0.5:
            # Gentle curve - keep speed high
            speed = base_speed * 0.9
            drive_mode = "GENTLE TURN"
        else:
            # Sharp curve - still maintain good speed (Task #2 fix)
            speed = base_speed * 0.75
            drive_mode = "SHARP TURN"

        if self.enabled:
            twist = Twist()
            if found:
                twist.linear.x = speed
                twist.angular.z = float(steering)
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.3
                drive_mode = "SEARCHING"
                self.get_logger().warn("LANE LOST - Searching...")
            
            self.cmd_pub.publish(twist)
        else:
            drive_mode = "MANUAL"

        # =============================================
        # F. DEBUG OVERLAY ON GUI (Task #1)
        # =============================================
        debug = frame.copy()
        
        # Draw ROI boundary line
        cv2.line(debug, (0, roi_top), (w, roi_top), (255, 255, 0), 2)
        cv2.putText(debug, "ROI", (5, roi_top - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        
        # Highlight detected pixels in ROI
        # Yellow detections in green
        yellow_overlay = cv2.cvtColor(roi_mask_y, cv2.COLOR_GRAY2BGR)
        yellow_overlay[:, :, 0] = 0; yellow_overlay[:, :, 2] = 0  # Keep only green
        debug[roi_top:] = cv2.addWeighted(debug[roi_top:], 0.8, yellow_overlay, 0.4, 0)
        
        # White detections in cyan
        white_overlay = cv2.cvtColor(roi_mask_w, cv2.COLOR_GRAY2BGR)
        white_overlay[:, :, 2] = 0  # Keep blue+green = cyan
        debug[roi_top:] = cv2.addWeighted(debug[roi_top:], 0.9, white_overlay, 0.3, 0)
        
        # Draw image center line (red)
        cv2.line(debug, (int(img_center), 0), (int(img_center), h), (0, 0, 255), 2)
        
        if found:
            # Draw target position (green dot)
            ty = roi_top + int((h - roi_top) * 0.5)
            cv2.circle(debug, (int(target_center), ty), 12, (0, 255, 0), -1)
            # Draw error arrow (yellow)
            cv2.arrowedLine(debug, (int(img_center), ty), (int(target_center), ty), (0, 255, 255), 3)
        
        # HUD Panel - dark background
        hud_h = 80
        overlay = debug.copy()
        cv2.rectangle(overlay, (0, h - hud_h), (w, h), (0, 0, 0), -1)
        debug = cv2.addWeighted(debug, 0.7, overlay, 0.3, 0)
        
        # HUD Text
        y_base = h - hud_h + 18
        color_green = (0, 255, 0) if found else (0, 0, 255)
        
        cv2.putText(debug, f"MODE: {detection_mode}", (10, y_base), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_green, 1)
        cv2.putText(debug, f"DRIVE: {drive_mode}", (10, y_base + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(debug, f"Error: {error:+.1f}px", (250, y_base), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(debug, f"Steer: {steering:+.3f}", (250, y_base + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(debug, f"Speed: {speed:.2f}", (430, y_base), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(debug, f"Kp: {kp}", (430, y_base + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
        
        # Steering bar indicator
        bar_cx = int(w / 2)
        bar_y = y_base + 45
        bar_w = 200
        cv2.rectangle(debug, (bar_cx - bar_w, bar_y - 6), (bar_cx + bar_w, bar_y + 6), (60, 60, 60), -1)
        steer_px = int(steering / 1.5 * bar_w)
        bar_color = (0, 255, 0) if abs(steer_px) < bar_w * 0.3 else (0, 200, 255) if abs(steer_px) < bar_w * 0.7 else (0, 0, 255)
        cv2.rectangle(debug, (bar_cx, bar_y - 6), (bar_cx + steer_px, bar_y + 6), bar_color, -1)
        cv2.line(debug, (bar_cx, bar_y - 10), (bar_cx, bar_y + 10), (255, 255, 255), 2)
        cv2.putText(debug, "L", (bar_cx - bar_w - 15, bar_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
        cv2.putText(debug, "R", (bar_cx + bar_w + 5, bar_y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
        
        # Publish to /lane_debug for monitoring
        self.vis_pub.publish(self.bridge.cv2_to_imgmsg(debug, "bgr8"))

def main():
    rclpy.init()
    rclpy.spin(LaneFollowerNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
