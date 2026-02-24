import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from flask import Flask, render_template_string, Response, request
import cv2
import threading
from cv_bridge import CvBridge
import time

# --- ROS2 Node for Backend ---
class RemoteNode(Node):
    def __init__(self):
        super().__init__('remote_control_backend')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.follow_pub = self.create_publisher(Bool, '/lane_following_enabled', 10)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 1)
        self.debug_sub = self.create_subscription(Image, '/lane_debug', self.debug_callback, 1)
        self.bridge = CvBridge()
        self.latest_frame = None
        self.debug_frame = None
        self.following = False

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            pass

    def debug_callback(self, msg):
        try:
            self.debug_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            pass

    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher.publish(msg)

# --- Flask App ---
app = Flask(__name__)
ros_node = None

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>Autonomous Vehicle Remote Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #0f172a; color: white; text-align: center; margin: 0; overflow-x: hidden; }
        .dashboard { display: grid; grid-template-columns: 1fr 350px; gap: 20px; padding: 20px; max-width: 1200px; margin: 0 auto; }
        .video-container { background: #1e293b; padding: 15px; border-radius: 15px; box-shadow: 0 10px 25px rgba(0,0,0,0.5); }
        .video-box { width: 100%; border-radius: 8px; overflow: hidden; border: 2px solid #3b82f6; }
        
        .control-panel { background: #1e293b; padding: 20px; border-radius: 15px; display: flex; flex-direction: column; gap: 15px; }
        #joystick-zone { width: 250px; height: 250px; background: #334155; border-radius: 50%; margin: 20px auto; position: relative; border: 4px solid #475569; }
        
        button { 
            background: #3b82f6; color: white; border: none; padding: 12px; border-radius: 8px; 
            font-weight: 600; cursor: pointer; transition: 0.2s; width: 100%; font-size: 14px;
        }
        button:hover { background: #2563eb; transform: translateY(-2px); }
        .btn-stop { background: #ef4444; margin-top: 10px; font-size: 16px; height: 50px; }
        .btn-stop:hover { background: #dc2626; }
        .btn-follow.active { background: #10b981; }
        
        .status-badge { background: #334155; padding: 10px; border-radius: 8px; font-size: 12px; font-family: monospace; color: #94a3b8; }
        h1 { margin: 0 0 20px 0; color: #3b82f6; font-weight: 800; letter-spacing: -1px; }
    </style>
</head>
<body>
    <div style="padding-top: 20px;">
        <h1>AGRIBOT CONTROL SYSTEM</h1>
    </div>
    
    <div class="dashboard">
        <div class="video-container">
            <div class="video-box">
                <img src="/video_feed" style="width: 100%; height: auto; display: block;">
            </div>
            <div style="margin-top: 15px; display: flex; justify-content: space-between; align-items: center;">
                <div class="status-badge" id="statusText">System: Standby</div>
                <button style="width: auto; padding: 8px 20px;" onclick="location.reload()">Refresh Stream</button>
            </div>
        </div>
        
        <div class="control-panel">
            <div style="font-weight: 600; font-size: 18px;">Analog Navigator</div>
            <div id="joystick-zone"></div>
            
            <button class="btn-stop" onclick="driveCustom(0, 0)">EMERGENCY STOP</button>
            <button id="followBtn" class="btn-follow" onclick="toggleFollow()">ACTIVATE AUTONOMOUS</button>
            
            <div class="status-badge">
                X: <span id="valX">0.00</span> | Z: <span id="valZ">0.00</span>
            </div>
        </div>
    </div>

    <script src="https://cdnjs.cloudflare.com/ajax/libs/nipplejs/0.10.1/nipplejs.min.js"></script>
    <script>
        var manager = nipplejs.create({
            zone: document.getElementById('joystick-zone'),
            mode: 'static',
            position: {left: '50%', top: '50%'},
            color: '#3b82f6',
            size: 150
        });

        let currentLin = 0;
        let currentAng = 0;

        manager.on('move', function (evt, data) {
            if (data.vector) {
                // Car-like controls:
                // Up = forward, Down = reverse
                // Left/Right = turn while moving forward (arc, not spin)
                let forward = data.vector.y;    // -1 to 1
                let steer = -data.vector.x;     // -1 to 1 (negative=right, positive=left)
                
                // If steering left/right, always add forward motion (like a real car)
                if (Math.abs(forward) < 0.15 && Math.abs(steer) > 0.1) {
                    forward = 0.4;  // Base forward speed during pure steering
                }
                
                currentLin = forward * 1.0;
                currentAng = steer * 1.5;
                
                document.getElementById('valX').innerText = currentLin.toFixed(2);
                document.getElementById('valZ').innerText = currentAng.toFixed(2);
                driveCustom(currentLin, currentAng);
            }
        });

        manager.on('end', function () {
            driveCustom(0, 0);
            document.getElementById('valX').innerText = "0.00";
            document.getElementById('valZ').innerText = "0.00";
        });

        function driveCustom(lin, ang) {
            fetch(`/move?lin=${lin}&ang=${ang}`);
        }

        function toggleFollow() {
            fetch('/toggle_follow').then(r => r.json()).then(data => {
                const btn = document.getElementById('followBtn');
                btn.innerText = data.following ? "DISABLE AI MODE" : "ACTIVATE AUTONOMOUS";
                btn.classList.toggle('active', data.following);
                document.getElementById('statusText').innerText = data.following ? "Status: Autonomous" : "Status: Manual";
            });
        }
    </script>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

def generate_frames():
    while True:
        # Prefer debug frame (with overlay) when lane follower is active
        display = ros_node.debug_frame if ros_node.debug_frame is not None else ros_node.latest_frame
        if display is not None:
            ret, buffer = cv2.imencode('.jpg', display)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.05)

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/move')
def move():
    lin = request.args.get('lin', 0)
    ang = request.args.get('ang', 0)
    ros_node.send_cmd(lin, ang)
    return "OK"

@app.route('/toggle_follow')
def toggle_follow():
    ros_node.following = not ros_node.following
    msg = Bool()
    msg.data = ros_node.following
    ros_node.follow_pub.publish(msg)
    return {"following": ros_node.following}

def ros_spin():
    rclpy.spin(ros_node)

if __name__ == '__main__':
    rclpy.init()
    ros_node = RemoteNode()
    
    # Start ROS in a separate thread
    threading.Thread(target=ros_spin, daemon=True).start()
    
    # Run Flask
    # Using 0.0.0.0 makes it accessible via the network IP
    app.run(host='0.0.0.0', port=5000)
