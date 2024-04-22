pkill gzserver
pkill gzclient

# download satellite images from google cloud
if [ -f "/workspace/rover/ros2/src/simulator/kiwi_gazebo/models/map_satellite_33.9698455_-118.416616_1500_1500/materials/textures/tile_33.9764296_-118.424556.png" ] 
then
    echo "Satellite images are already in your system. Will not download" 
else
    echo "Satellite images not found in your system. Downloading them ..."
    gsutil -m cp -r gs://gazebo_satellite_images/satellite_images/* /workspace/rover/ros2/src/simulator/kiwi_gazebo/models/
fi

cd rover/ros2/
colcon build --symlink-install



export GAZEBO_PLUGIN_PATH=/workspace/rover/ros2/build/kiwi_gazebo:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:
export GAZEBO_MODEL_PATH=/workspace/rover/ros2/src/simulator/kiwi_gazebo/models/:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=/workspace/rover/ros2/src/simulator/kiwi_gazebo/models/:$GAZEBO_RESOURCE_PATH

 #gzserver /workspace/rover/ros2/src/simulator/kiwi_gazebo/worlds/3_LMU.world --verbose & 
 #sleep 5 
source install/setup.bash 
ros2 launch kiwi_gazebo LMU.launch.py
#gazebo /workspace/rover/ros2/src/simulator/kiwi_gazebo/worlds/3_LMU.world --verbose -s libgazebo_ros_factory.so -g libpolygon_parser.so
#gzclient -g libpolygon_parser.so 


