import xml.etree.ElementTree as ET
import math
import os

# Center point for (0,0) in Gazebo
CENTER_LAT = 15.3695
CENTER_LON = 75.123

# WGS84 Projection Constants
R = 6378137.0 # Earth radius in meters

def latlon_to_meters(lat, lon):
    # Mercator meters projection relative to center
    y = (lat - CENTER_LAT) * (math.pi / 180.0) * R
    x = (lon - CENTER_LON) * (math.pi / 180.0) * R * math.cos(math.radians(CENTER_LAT))
    return x, y

osm_path = '/home/arihant/Music/college/planet_75.118,15.366_75.128,15.373 (1).osm'
tree = ET.parse(osm_path)
root = tree.getroot()

nodes = {}
for node in root.findall('node'):
    nodes[node.get('id')] = (float(node.get('lat')), float(node.get('lon')))

buildings = []
for way in root.findall('way'):
    is_building = False
    for tag in way.findall('tag'):
        if tag.get('k') == 'building':
            is_building = True
            break
    
    if is_building:
        way_nodes = [nd.get('ref') for nd in way.findall('nd')]
        pts = []
        for nid in way_nodes:
            if nid in nodes:
                pts.append(latlon_to_meters(*nodes[nid]))
        if pts:
            buildings.append({'id': way.get('id'), 'pts': pts})

# Create the SDF
sdf_head = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="osm_buildings">
    <static>true</static>
    <link name="link">
"""

sdf_body = ""
for i, b in enumerate(buildings):
    pts_str = "\n".join([f"            <point>{p[0]} {p[1]}</point>" for p in b['pts']])
    sdf_body += f"""
      <visual name="building_{b['id']}_{i}">
        <geometry>
          <polyline>
{pts_str}
            <height>8.0</height>
          </polyline>
        </geometry>
        <material>
          <ambient>0.6 0.6 0.6 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
        </material>
      </visual>
      <collision name="col_{b['id']}_{i}">
        <geometry>
          <polyline>
{pts_str}
            <height>8.0</height>
          </polyline>
        </geometry>
      </collision>
"""

sdf_foot = """
    </link>
  </model>
</sdf>
"""

model_dir = "/home/arihant/ros2_ws/src/lane_simulation/models/osm_buildings"
os.makedirs(model_dir, exist_ok=True)
with open(os.path.join(model_dir, "model.sdf"), "w") as f:
    f.write(sdf_head + sdf_body + sdf_foot)

print(f"Generated {len(buildings)} buildings aligned to meter-coordinates.")
