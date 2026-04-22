import numpy as np

def generate_lidar_xml(num_beams=360):
    header = """<mujoco model="Lite3_with_Lidar">
  <default>
    <default class="robot">
      <default class="motor">
        <joint />
        <motor />
      </default>
      <default class="visual">
        <geom material="visualgeom" contype="0" conaffinity="0" group="2" />
      </default>
      <default class="collision">
        <geom material="collision_material" condim="3" contype="1" conaffinity="1" group="1" solref="0.005 1" friction="1 0.01 0.01" />
      </default>
    </default>
  </default>

  <compiler angle="radian" meshdir="./meshes/" />
  <compiler autolimits="true"/>

  <visual>
    <global offwidth="1920" offheight="1080" />
  </visual>

  <asset>
    <material name="default_material" rgba="0.7 0.7 0.7 1" />
    <material name="visualgeom" rgba="0.5 0.5 0.5 1" />
    <material name="collision_material" rgba="0.0 0.4 0.8 0.2" />
    <texture type="2d" name="checker" builtin="checker" rgb1="0.2 0.3 0.4" rgb2="0.9 0.9 0.9" width="500" height="500"/>
    <material name="checker_mat" texture="checker" specular="1" shininess="1" texrepeat="15 15"/>
    <mesh name="torso.STL" file="torso.STL" />
    <mesh name="fl_hip.STL" file="hip.STL" />
    <mesh name="fl_thigh.STL" file="thigh.STL" />
    <mesh name="fl_shank_collision.STL" file="shank.STL" />
    <mesh name="fr_hip.STL" file="hip.STL" />
    <mesh name="fr_thigh.STL" file="thigh.STL" scale="1 -1 1"/>
    <mesh name="fr_shank_collision.STL" file="shank.STL" />
    <mesh name="hl_hip.STL" file="hip.STL" />
    <mesh name="hl_thigh.STL" file="thigh.STL" />
    <mesh name="hl_shank_collision.STL" file="shank.STL" />
    <mesh name="hr_hip.STL" file="hip.STL" />
    <mesh name="hr_thigh.STL" file="thigh.STL" scale="1 -1 1"/>
    <mesh name="hr_shank_collision.STL" file="shank.STL" />
  </asset>

  <worldbody>
    <geom name="floor" type="plane" size="10 10 0.1" pos="0 0 0" material="checker_mat" condim="3" contype="1" conaffinity="1"/>
    <light directional="true" pos="-0.5 0.5 3" dir="0 0 -1"/>

    <geom name="wall_north" type="box" size="5 0.1 1.0" pos="0 5 1.0" rgba="0.8 0.5 0.5 1"/>
    <geom name="wall_south" type="box" size="5 0.1 1.0" pos="0 -5 1.0" rgba="0.8 0.5 0.5 1"/>
    <geom name="wall_east" type="box" size="0.1 5 1.0" pos="5 0 1.0" rgba="0.5 0.8 0.5 1"/>
    <geom name="wall_west" type="box" size="0.1 5 1.0" pos="-5 0 1.0" rgba="0.5 0.8 0.5 1"/>
    
    <geom name="box_1" type="box" size="0.2 0.2 1.0" pos="1 1 1.0" rgba="0.5 0.5 0.8 1"/>
    <geom name="box_2" type="box" size="0.3 0.3 1.0" pos="-1.5 0.5 1.0" rgba="0.8 0.8 0.2 1"/>

    <body name="TORSO" pos="0 0. 0.5" quat="1 0 0 0" childclass="robot">
      <freejoint name="floating_base" />
      <site name="imu_site" pos="0 0 0" size="0.005" rgba="1 0 0 1"/>
      <inertial pos="0 0 0" mass="5.6056" diaginertia="0.02456 0.05518 0.07016"/>
      <geom name="TORSO_collision" type="mesh" mesh="torso.STL" class="collision" />
      <geom name="TORSO_visual" type="mesh" mesh="torso.STL" class="visual" />
      
      <body name="lidar_mount" pos="0.15 0 0.07">
        <geom type="cylinder" size="0.03 0.02" rgba="0.1 0.1 0.1 1" />
"""

    footer_camera = """
      </body>

      <!-- Front-facing Vision Suite (RGB + Depth) -->
      <body name="vision_mount" pos="0.25 0 0.05" quat="1 0 0 0">
        <geom type="box" size="0.01 0.04 0.02" rgba="0.2 0.2 0.2 1" />
        <camera name="front_vision_camera" pos="0.01 0 0" quat="0.5 -0.5 0.5 -0.5" fovy="60" />
      </body>
"""

    footer_legs = """
      <body name="FL_HIP" pos="0.1745 0.062 0">
        <joint name="FL_HipX_joint" type="hinge" ref="0.0" class="motor" range="-0.523 0.523" axis="-1 0 0" />
        <geom name="FL_HIP_visual" type="mesh" mesh="fl_hip.STL" class="visual" />
        <body name="FL_THIGH" pos="0 0.09735 0">
          <joint name="FL_HipY_joint" type="hinge" ref="0.0" class="motor" range="-2.67 0.314" axis="0 -1 0" />
          <geom name="FL_THIGH_visual" type="mesh" mesh="fl_thigh.STL" class="visual" />
          <geom name="FL_THIGH_collision" type="box" size="0.11 0.02 0.02" pos="0 0.015 -0.1" class="collision" />
          <body name="FL_SHANK" pos="0 0 -0.20">
            <joint name="FL_Knee_joint" type="hinge" ref="0.0" class="motor" range="0.524 2.792" axis="0 -1 0" />
            <geom name="FL_SHANK_visual" type="mesh" mesh="fl_shank_collision.STL" class="visual" />
            <geom name="FL_SHANK_collision" type="box" size="0.015 0.01 0.1" pos="0 0 -0.1" class="collision" />
            <body name="FL_FOOT" pos="0 0 -0.21012">
              <geom name="FL_FOOT_collision" type="sphere" size="0.022" class="collision" />
            </body>
          </body>
        </body>
      </body>
      <body name="FR_HIP" pos="0.1745 -0.062 0">
        <joint name="FR_HipX_joint" type="hinge" ref="0.0" class="motor" range="-0.523 0.523" axis="-1 0 0" />
        <geom name="FR_HIP_visual" type="mesh" mesh="fr_hip.STL" class="visual" />
        <body name="FR_THIGH" pos="0 -0.09735 0">
          <joint name="FR_HipY_joint" type="hinge" ref="0.0" class="motor" range="-2.67 0.314" axis="0 -1 0" />
          <geom name="FR_THIGH_visual" type="mesh" mesh="fr_thigh.STL" class="visual" />
          <geom name="FR_THIGH_collision" type="box" size="0.11 0.02 0.02" pos="0 -0.015 -0.1" class="collision" />
          <body name="FR_SHANK" pos="0 0 -0.20">
            <joint name="FR_Knee_joint" type="hinge" ref="0.0" class="motor" range="0.524 2.792" axis="0 -1 0" />
            <geom name="FR_SHANK_visual" type="mesh" mesh="fr_shank_collision.STL" class="visual" />
            <geom name="FR_SHANK_collision" type="box" size="0.015 0.01 0.1" pos="0 0 -0.1" class="collision" />
            <body name="FR_FOOT" pos="0 0 -0.21012">
              <geom name="FR_FOOT_collision" type="sphere" size="0.022" class="collision" />
            </body>
          </body>
        </body>
      </body>
      <body name="HL_HIP" pos="-0.1745 0.062 0">
        <joint name="HL_HipX_joint" type="hinge" ref="0.0" class="motor" range="-0.523 0.523" axis="-1 0 0" />
        <geom name="HL_HIP_visual" type="mesh" mesh="hl_hip.STL" class="visual" />
        <body name="HL_THIGH" pos="0 0.09735 0">
          <joint name="HL_HipY_joint" type="hinge" ref="0.0" class="motor" range="-2.67 0.314" axis="0 -1 0" />
          <geom name="HL_THIGH_visual" type="mesh" mesh="hl_thigh.STL" class="visual" />
          <geom name="HL_THIGH_collision" type="box" size="0.11 0.02 0.02" pos="0 0.015 -0.1" class="collision" />
          <body name="HL_SHANK" pos="0 0 -0.20">
            <joint name="HL_Knee_joint" type="hinge" ref="0.0" class="motor" range="0.524 2.792" axis="0 -1 0" />
            <geom name="HL_SHANK_visual" type="mesh" mesh="hl_shank_collision.STL" class="visual" />
            <geom name="HL_SHANK_collision" type="box" size="0.015 0.01 0.1" pos="0 0 -0.1" class="collision" />
            <body name="HL_FOOT" pos="0 0 -0.21012">
              <geom name="HL_FOOT_collision" type="sphere" size="0.022" class="collision" />
            </body>
          </body>
        </body>
      </body>
      <body name="HR_HIP" pos="-0.1745 -0.062 0">
        <joint name="HR_HipX_joint" type="hinge" ref="0.0" class="motor" range="-0.523 0.523" axis="-1 0 0" />
        <geom name="HR_HIP_visual" type="mesh" mesh="hr_hip.STL" class="visual" />
        <body name="HR_THIGH" pos="0 -0.09735 0">
          <joint name="HR_HipY_joint" type="hinge" ref="0.0" class="motor" range="-2.67 0.314" axis="0 -1 0" />
          <geom name="HR_THIGH_visual" type="mesh" mesh="hr_thigh.STL" class="visual" />
          <geom name="HR_THIGH_collision" type="box" size="0.11 0.02 0.02" pos="0 -0.015 -0.1" class="collision" />
          <body name="HR_SHANK" pos="0 0 -0.20">
            <joint name="HR_Knee_joint" type="hinge" ref="0.0" class="motor" range="0.524 2.792" axis="0 -1 0" />
            <geom name="HR_SHANK_visual" type="mesh" mesh="hr_shank_collision.STL" class="visual" />
            <geom name="HR_SHANK_collision" type="box" size="0.015 0.01 0.1" pos="0 0 -0.1" class="collision" />
            <body name="HR_FOOT" pos="0 0 -0.21012">
              <geom name="HR_FOOT_collision" type="sphere" size="0.022" class="collision" />
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>
  
  <actuator>
    <motor name="FL_HipX_joint_ctrl" joint="FL_HipX_joint" class="motor" gear="1" ctrlrange="-30 30"/>
    <motor name="FL_HipY_joint_ctrl" joint="FL_HipY_joint" class="motor" ctrlrange="-30 30"/>
    <motor name="FL_Knee_joint_ctrl" joint="FL_Knee_joint" class="motor" ctrlrange="-30 30"/>
    <motor name="FR_HipX_joint_ctrl" joint="FR_HipX_joint" class="motor" gear="1" ctrlrange="-30 30"/>
    <motor name="FR_HipY_joint_ctrl" joint="FR_HipY_joint" class="motor" ctrlrange="-30 30"/>
    <motor name="FR_Knee_joint_ctrl" joint="FR_Knee_joint" class="motor" ctrlrange="-30 30"/>
    <motor name="HL_HipX_joint_ctrl" joint="HL_HipX_joint" class="motor" gear="1" ctrlrange="-30 30"/>
    <motor name="HL_HipY_joint_ctrl" joint="HL_HipY_joint" class="motor" ctrlrange="-30 30"/>
    <motor name="HL_Knee_joint_ctrl" joint="HL_Knee_joint" class="motor" ctrlrange="-30 30"/>
    <motor name="HR_HipX_joint_ctrl" joint="HR_HipX_joint" class="motor" gear="1" ctrlrange="-30 30"/>
    <motor name="HR_HipY_joint_ctrl" joint="HR_HipY_joint" class="motor" ctrlrange="-30 30"/>
    <motor name="HR_Knee_joint_ctrl" joint="HR_Knee_joint" class="motor" ctrlrange="-30 30"/>
  </actuator>

  <sensor>
"""

    sites = ""
    sensors = ""
    radius = 0.04 
    for i in range(num_beams):
        angle = (2 * np.pi * i) / num_beams
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        z = 0.02
        vx = np.cos(angle)
        vy = np.sin(angle)
        vz = 0
        sites += f'        <site name="ray_{i}" pos="{x:.4f} {y:.4f} {z:.4f}" zaxis="{vx:.4f} {vy:.4f} {vz:.4f}" />\n'
        sensors += f'    <rangefinder name="lidar_{i}" site="ray_{i}" />\n'

    full_xml = header + sites + footer_camera + footer_legs + sensors + "\n  </sensor>\n</mujoco>"
    
    with open("lite3_lidar.xml", "w") as f:
        f.write(full_xml)
    print(f"Generated lite3_lidar.xml with {num_beams} beams.")

if __name__ == "__main__":
    generate_lidar_xml(360)
