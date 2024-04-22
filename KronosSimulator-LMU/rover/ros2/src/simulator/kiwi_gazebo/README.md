# Kiwi Gazebo Package
Powered by Gazebo Simulator

---

### Code Information
**Maintainer:** _[Eng. Davidson Daniel Rojas Cediel](https://www.linkedin.com/in/dadaroce/)_ 

**Mail:** _davidson@kiwibot.com_ 

**Kiwi Campus / AI & Robotics Team**

---
---

<p align="center">
<img src="https://user-images.githubusercontent.com/39452483/107431798-f468b780-6af4-11eb-998c-6fdf14828f86.png" />
</p> 


<p align="center">
Simulated Kiwibot
</p>


---
---


## Why Gazebo?
Robot simulation is an essential tool in every roboticist's toolbox. A well-designed simulator makes it possible to rapidly test algorithms, design robots, perform regression testing, and train AI system using realistic scenarios. Gazebo offers the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. At your fingertips is a robust physics engine, high-quality graphics, and convenient programmatic and graphical interfaces. Best of all, Gazebo is free with a vibrant community.
> _Source [Gazebo Simulator Page](http://gazebosim.org/)_

---


```
File Tree 
📦kiwi_gazebo
 ┣ 📂launch
 ┃ ┣ 📜kiwi.launch.py
 ┃ ┗ 📜sim_stack.launch.py
 ┣ 📂meshes
 ┃ ┣ 📜Bed.dae
 ┃ ┣ 📜black_chair.dae
 ┃ ┣ 📜cuadrante_1_v2.obj
 ┃ ┣ 📜door.dae
 ┃ ┣ 📜fridge.dae
 ┃ ┣ 📜kitchen.dae
 ┃ ┣ 📜kiwibot4.dae
 ┃ ┣ 📜outside_couch.dae
 ┃ ┣ 📜plant.dae
 ┃ ┗ 📜toilet.dae
 ┣ 📂models
 ┃ ┣ 📂Cuadrante_1_LMU
 ┃ ┃ ┣ 📜model.config
 ┃ ┃ ┗ 📜model.sdf
 ┃ ┣ 📂bed
 ┃ ┃ ┣ 📜model.config
 ┃ ┃ ┗ 📜model.sdf
 ┃ ┣ 📂black_chair
 ┃ ┃ ┣ 📜model.config
 ┃ ┃ ┗ 📜model.sdf
 ┃ ┣ 📂city_terrain
 ┃ ┃ ┣ 📂materials
 ┃ ┃ ┃ ┗ 📂textures
 ┃ ┃ ┃ ┃ ┣ 📜city_terrain (original).jpg
 ┃ ┃ ┃ ┃ ┣ 📜city_terrain.jpg
 ┃ ┃ ┃ ┃ ┗ 📜forest.jpg
 ┃ ┃ ┣ 📜model.config
 ┃ ┃ ┗ 📜model.sdf
 ┃ ┣ 📂door
 ┃ ┃ ┣ 📜model.config
 ┃ ┃ ┗ 📜model.sdf
 ┃ ┣ 📂fridge
 ┃ ┃ ┣ 📜model.config
 ┃ ┃ ┗ 📜model.sdf
 ┃ ┣ 📂kitchen
 ┃ ┃ ┣ 📜model.config
 ┃ ┃ ┗ 📜model.sdf
 ┃ ┣ 📂kiwi
 ┃ ┃ ┣ 📜model.config
 ┃ ┃ ┗ 📜model.sdf
 ┃ ┣ 📂ocean
 ┃ ┃ ┣ 📂materials
 ┃ ┃ ┃ ┣ 📂programs
 ┃ ┃ ┃ ┃ ┗ 📂GLSL
 ┃ ┃ ┃ ┃ ┃ ┣ 📜Waves.frag
 ┃ ┃ ┃ ┃ ┃ ┗ 📜Waves.vert
 ┃ ┃ ┃ ┣ 📂scripts
 ┃ ┃ ┃ ┃ ┗ 📜ocean.material
 ┃ ┃ ┃ ┗ 📂textures
 ┃ ┃ ┃ ┃ ┣ 📜License and source soil_sand_0045_01.txt
 ┃ ┃ ┃ ┃ ┣ 📜clouds.jpg
 ┃ ┃ ┃ ┃ ┣ 📜clouds_bk.jpg
 ┃ ┃ ┃ ┃ ┣ 📜clouds_dn.jpg
 ┃ ┃ ┃ ┃ ┣ 📜clouds_fr.jpg
 ┃ ┃ ┃ ┃ ┣ 📜clouds_lf.jpg
 ┃ ┃ ┃ ┃ ┣ 📜clouds_rt.jpg
 ┃ ┃ ┃ ┃ ┣ 📜clouds_up.jpg
 ┃ ┃ ┃ ┃ ┣ 📜normals.dds
 ┃ ┃ ┃ ┃ ┗ 📜soil_sand_0045_01.jpg
 ┃ ┃ ┣ 📜mesh-simple.dae
 ┃ ┃ ┣ 📜mesh.blend
 ┃ ┃ ┣ 📜mesh.dae
 ┃ ┃ ┣ 📜mesh_below.dae
 ┃ ┃ ┣ 📜model.config
 ┃ ┃ ┗ 📜model.sdf
 ┃ ┣ 📂outside_couch
 ┃ ┃ ┣ 📜model.config
 ┃ ┃ ┗ 📜model.sdf
 ┃ ┗ 📂toilet
 ┃ ┃ ┣ 📜model.config
 ┃ ┃ ┗ 📜model.sdf
 ┣ 📂worlds
 ┃ ┣ 📜0_dolly_empty.world
 ┃ ┣ 📜1_dolly_city.world
 ┃ ┣ 📜2_mundo_medellin.world
 ┃ ┣ 📜3_LMU.world
 ┃ ┗ 📜dolly_city_sidewalk_obstacles.world
 ┣ 📜CMakeLists.txt
 ┣ 📜README.md
 ┗ 📜package.xml
```

---

## Node Responsibilities

This node is shown as **kiwi_gazebo**. It is in charge of:

- Provide a Debug and Developed tool to fr the entire Stack of the Kronos Project

## Node functionality
In order to launch simulated environment it's recommended to use the below steps: 

```.bash
$ cd /workspace/scripts
$ bash startSimulator.sh
```

> Note: Mandatory to use the Script inside the provided Path

### Script Steps

**1.** Download the whole 3D Models used by the Sim-City Simulator, such of them are placed inside the ```/home/ada/.gazebo/models/``` folder to improve First Launch performance. In case you'll not use the city, just change the map with the flag `-c` or `--city`: 

```.bash
$ bash startSimulator.sh -c [world ID]
```

**2.** Source the Environment Parameters such like ```ROS_DOMAIN_ID``` in order to isolate the simulator ROS2 environment. Besides, source the Gazebo 11, Galactic Gazebo (previously 9 used by Dashing) and ROS2 source.

**3.** Killing previous Client and Server to avoid launch several Gazebo screens that could overwhelm the CPU/GPU.

**4.** Build Simulated Devices and enable transport between Gazebo Topics and ROS2 **Topics**.

**5.** Launch The Simulator either use or not the sim-city.

---
---
## Generated Topics


### Publishers

- LIDAR

    Topic: 
    
        /scan

    Brief: 
        
        Publish 2D scan Pointcloud

    Type: 
        
        sensor_msgs/LaserScan

    Rate: 
        
        15
    \
    Implemented using: 

```.xml
        <plugin name="laser" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <namespace>/</namespace>
            <argument>~/out:=scan</argument>
          </ros>
          <output_type>sensor_msgs/LaserScan</output_type>
        </plugin>
```
\
    Related to: \
        _[Gazebo ROS2 Laser Migration](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Ray-sensors#gazebo_ros_laser)_

---

- IMU sensor

    Topic: 

        /imu/data
    
    Brief: 
    
        Publish 2D scan pointcloud

    Type: 
    
        sensor_msgs/Imu

    Rate: 
    
        30
    \
    Implemented using: 

```.xml
    <plugin name="my_imu_plugin" filename="libgazebo_ros_imu_sensor.so">
        <ros>
        <namespace>/imu</namespace>
        <argument>~/out:=data</argument>
        </ros>
    </plugin>
``` 
\
    Related to:\
        _[Gazebo ROS IMU Migration](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-IMU-Sensors)_

---

- Cameras

    Topics: 

        /gazebo/streaming_cam_rear/image_raw
        /gazebo/streaming_cam_rear/camera_info
        /gazebo/streaming_cam_right/image_raw
        /gazebo/streaming_cam_right/camera_info
        /gazebo/streaming_cam_left/image_raw
        /gazebo/streaming_cam_left/camera_info
        /gazebo/streaming_cam_zoom/image_raw
        /gazebo/streaming_cam_zoom/camera_info

    Brief:

        Publish Image and Camera Info 

    Types: 

        sensor_msgs/Image
        sensor_msgs/CameraInfo 

    Rate: 
            
        30
    \
    Implemented using: 

```.xml
    <plugin name="streaming_cam_name" filename="libgazebo_ros_camera.so">
    <ros>
        <namespace>gazebo</namespace>
        <argument>image_raw:=image_raw</argument>
        <argument>camera_info:=camera_info</argument>
    </ros>
    <hack_baseline>0.07</hack_baseline>
    </plugin>
```
\
    Related to:\
        _[Gazebo ROS2 Camera](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Camera#gazebo_ros_camera)_

---

- Camera Stereo

    Topics: 
    
        /camera/color/image_raw
        /camera/aligned_depth_to_color/image_raw
        /camera/color/image_info
        /camera/depth/color/points_info
        /camera/color/aligned_depth_to_color/color/points


    Brief: 
    
        Publish Image, Depth Image and Stereo Camera Info 

    Types: 

        sensor_msgs/Image
        sensor_msgs/CameraInfo 
        sensor_msgs/PointCloud

    Rate: 
    
        1
    \
    Implemented using: 

```.xml
    <plugin name="stereo_plugin" filename="libgazebo_ros_camera.so">
        <ros>
        <namespace>camera</namespace> 
        <argument>camera/image_raw:=color/image_raw</argument>
        <argument>camera/depth/image_raw:=aligned_depth_to_color/image_raw</argument>
        <argument>camera/camera_info_raw:=color/image_info</argument>
        <argument>camera/depth/camera_info:=aligned_depth_to_color/color/points_info</argument>
        <argument>camera/points:=aligned_depth_to_color/color/points</argument>
        </ros>
        <!-- Set camera name. If empty, defaults to sensor name (i.e. "sensor_name") -->
        <!-- <camera_name>camera</camera_name> -->
        <!-- Set TF frame name. If empty, defaults to link name (i.e. "link_name") -->
        <frame_name>chassis</frame_name>
        <hack_baseline>0.07</hack_baseline>
        <min_depth>0.001</min_depth>
    </plugin>
```
\
    Related to:\
        _[Gazebo ROS2 Stereo Camera](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Camera#gazebo_ros_multicamera)_

---

- Wheels

    Topics: 
    
        /local_client/sim_control
        /wheel_odometry/global_odometry


    Brief: 
    
        Publish Velocities and wheel Odometry  

    Types: 
    
        geometry_msgs/Twist
        nav_msgs/Odometry 

    Rate: 
    
        500
    \
    Implemented using: 

```.xml
    <plugin name='skid_steer_drive' filename='libgazebo_ros_diff_drive.so'>
      <ros>
        <!-- Set namespace -->
        <namespace>/</namespace>

        <!-- Remap default topics -->
        <argument>cmd_vel:=local_client/sim_control</argument>
        <argument>odom:=odom</argument>
      </ros>

      <!-- Update rate -->
      <update_rate>60</update_rate>

      <!-- Number of wheel pairs -->
      <num_wheel_pairs>2</num_wheel_pairs>
      
      <!-- wheels0 -->
      <left_joint>left_wheel_4</left_joint>
      <right_joint>left_wheel_2</right_joint>

      <!-- wheels1-->
      <left_joint>left_wheel_3</left_joint>
      <right_joint>left_wheel_1</right_joint>

      <!-- kinematics -->
      <wheel_separation>0.36</wheel_separation>
      <wheel_separation>0.36</wheel_separation>

      <wheel_diameter>0.14</wheel_diameter>
      <wheel_diameter>0.14</wheel_diameter>

      <!-- limits -->
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>

      <!-- output -->
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>

      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>chassis</robot_base_frame>

    </plugin>
```
\
    Related to:\
        _[Gazebo ROS2 Skid Steer Drive](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Skid-Steer-drive)_

---

- TFMini Plus

    Topics: 
    
        /tf_lidar/rangeX [replace X with numbers from 0 to 4]

    Brief: 
    
        Publish proximity sensor data  

    Types: 
    
        sensor_msgs/Range

    Rate: 
    
        15
    \
    Implemented using: 

```.xml
    <plugin name="range_sensor" filename="libgazebo_ros_ray_sensor.so">
        <ros>
        <namespace>/tf_lidar</namespace>
        <argument>~/out:=rangeX</argument>
        </ros>
        <output_type>sensor_msgs/Range</output_type>
    </plugin>
```
\
    Related to:\
        _[Gazebo ROS2 Camera](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Camera#gazebo_ros_camera)_

---

### Subscribers

---

- Screen Video

    Topic: 
    
        /device/screen_image/image_raw

    Brief: 
    
        Subscribe to a Image topic in order to get a Video to put on the Simulated Screen

    Types: 
    
        sensor_msgs/Image
    \
    Implemented using: 

```.xml
    <plugin name="display_video_controller" filename="libgazebo_ros_video.so">
        <ros>
        <argument>~/image_raw:=/device/screen_image/image_raw</argument>
        </ros>
        <height>960</height>
        <width>1920</width>
    </plugin>
```
\
    Related to:\
        _[Gazebo ROS2 Video Plugin](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Video)_

---