#!/usr/bin/env python3
import os
import random

def generate_world():
    # Configuration
    rows = 6        # Even number ensures gap at y=0 if centered
    row_length = 20.0
    row_spacing = 2.0
    plant_spacing = 0.6 # Denser plants
    
    # Header
    world_content = """<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="custom_maize_farm">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Custom Grass Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grass</name>
            </script>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Physics settings -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Geographic coordinates (Hubli, India) -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>15.3478</latitude_deg>
      <longitude_deg>75.1338</longitude_deg>
      <elevation>0.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

"""

    # Generate Rows
    # Center the field at 0,0
    # For rows=6, spacing=2:
    # start_y = -((6-1)*2)/2 = -5
    # Rows at: -5, -3, -1, 1, 3, 5
    # Gaps at: -4, -2, 0, 2, 4
    start_y = -((rows - 1) * row_spacing) / 2
    
    plant_count = 0
    plant_locations = []
    
    for r in range(rows):
        y = start_y + (r * row_spacing)
        start_x = -row_length / 2
        
        # Calculate number of plants in this row
        num_plants_in_row = int(row_length / plant_spacing)
        
        for p in range(num_plants_in_row):
            x = start_x + (p * plant_spacing)
            
            # Add some noise to position
            x += random.uniform(-0.05, 0.05)
            y_noise = random.uniform(-0.05, 0.05)
            final_y = y + y_noise
            yaw = random.uniform(0, 6.28)
            
            plant_locations.append((x, final_y))
            
            # Force using maize_01
            model_type = "maize_01" 
            
            plant_xml = f"""
    <include>
      <uri>model://{model_type}</uri>
      <name>plant_{r}_{p}</name>
      <pose>{x} {final_y} 0 0 0 {yaw}</pose>
      <static>true</static>
    </include>
"""
            world_content += plant_xml
            plant_count += 1

    # Footer
    world_content += """
  </world>
</sdf>
"""

    output_path = "src/farm_sim/worlds/custom_maize.world"
    with open(output_path, "w") as f:
        f.write(world_content)
    
    print(f"Generated {output_path} with {plant_count} plants.")
    
    # Save coordinates to CSV
    csv_path = "crop_coordinates.csv"
    with open(csv_path, "w") as f:
        f.write("x,y\n")
        for px, py in plant_locations:
            f.write(f"{px},{py}\n")
    print(f"Saved coordinates to {csv_path}")

if __name__ == "__main__":
    generate_world()
