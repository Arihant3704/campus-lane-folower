import xml.etree.ElementTree as ET
import cv2
import numpy as np
import math
import os

# SAME constants as building script for perfect alignment
CENTER_LAT = 15.3695
CENTER_LON = 75.123
R = 6378137.0

# OSM Bounds to cover
min_lat, max_lat = 15.366, 15.373
min_lon, max_lon = 75.118, 75.128

def latlon_to_meters(lat, lon):
    y = (lat - CENTER_LAT) * (math.pi / 180.0) * R
    x = (lon - CENTER_LON) * (math.pi / 180.0) * R * math.cos(math.radians(CENTER_LAT))
    return x, y

# Calculate meter extent for the image
min_x, min_y = latlon_to_meters(min_lat, min_lon)
max_x, max_y = latlon_to_meters(max_lat, max_lon)

width_m = max_x - min_x
height_m = max_y - min_y
scale = 10.0  # HIGH RESOLUTION (10 px/m)

img_w = int(width_m * scale)
img_h = int(height_m * scale)

def meters_to_pixel(x, y):
    px = int((x - min_x) * scale)
    py = int((max_y - y) * scale) # Invert Y for image
    return px, py

osm_path = '/home/arihant/Music/college/planet_75.118,15.366_75.128,15.373 (1).osm'
tree = ET.parse(osm_path)
root = tree.getroot()

nodes = {}
for node in root.findall('node'):
    nodes[node.get('id')] = (float(node.get('lat')), float(node.get('lon')))

ways = []
for way in root.findall('way'):
    if any(tg.get('k') == 'highway' for tg in way.findall('tag')):
        ways.append([nd.get('ref') for nd in way.findall('nd')])

# Create transparent image
img = np.zeros((img_h, img_w, 4), dtype=np.uint8)

# Colors (B, G, R, A)
ASPHALT_COLOR = (10, 10, 10, 255) # Deep black
WHITE_CENTER = (255, 255, 255, 255)
YELLOW_EDGE = (0, 210, 255, 255) # Bright Yellow

ROAD_WIDTH_M = 3.5 # Optimized for ebot scale

# Draw Roads
for way_nodes in ways:
    pts = []
    for nid in way_nodes:
        if nid in nodes:
            x, y = latlon_to_meters(*nodes[nid])
            pts.append(meters_to_pixel(x, y))
    
    if len(pts) > 1:
        pts_arr = np.array(pts, np.int32).reshape((-1, 1, 2))
        
        # 1. Draw Asphalt
        cv2.polylines(img, [pts_arr], False, ASPHALT_COLOR, int(ROAD_WIDTH_M * scale))
        
        # 2. Draw Yellow Edges (Sharp and crisp)
        cv2.polylines(img, [pts_arr], False, YELLOW_EDGE, int((ROAD_WIDTH_M - 0.1) * scale))
        cv2.polylines(img, [pts_arr], False, ASPHALT_COLOR, int((ROAD_WIDTH_M - 0.3) * scale))

        # 3. Draw Dashed Center White Line (Sharp segments)
        for i in range(len(pts) - 1):
            p1, p2 = np.array(pts[i]), np.array(pts[i+1])
            dist = np.linalg.norm(p2 - p1)
            if dist < 5: continue
            
            num_steps = int(dist / (2.0 * scale)) # dash every 2 meters
            for s in range(0, num_steps, 2):
                s1 = p1 + (p2 - p1) * (s / num_steps)
                s2 = p1 + (p2 - p1) * (min(s+1, num_steps) / num_steps)
                cv2.line(img, tuple(s1.astype(int)), tuple(s2.astype(int)), WHITE_CENTER, int(0.12 * scale))

# Save Texture
save_dir = '/home/arihant/ros2_ws/src/lane_simulation/models/college_road_network/materials/textures'
os.makedirs(save_dir, exist_ok=True)
cv2.imwrite(os.path.join(save_dir, 'road_overlay.png'), img)

# Update the Model SDF with correct plane size
sdf_path = '/home/arihant/ros2_ws/src/lane_simulation/models/college_road_network/model.sdf'
sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="college_road_network">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>{width_m} {height_m}</size>
          </plane>
        </geometry>
        <material>
          <script>
            <uri>model://college_road_network/materials/scripts</uri>
            <uri>model://college_road_network/materials/textures</uri>
            <name>RoadOverlay</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
with open(sdf_path, 'w') as f: f.write(sdf_content)

print(f"High-res road texture (Scale: {scale}, Width: {ROAD_WIDTH_M}m) and SDF updated.")
