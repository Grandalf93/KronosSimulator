import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from scripts import GazeboRosPaths


def generate_launch_description():
    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()

    env = {
        "GAZEBO_MODEL_PATH": model_path,
        "GAZEBO_PLUGIN_PATH": plugin_path,
        "GAZEBO_RESOURCE_PATH": media_path,
    }

  

    world_prefix = get_package_share_directory("kiwi_gazebo")
    world_file = os.path.join(world_prefix, "worlds", "3_LMU.world")

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[
                    "gazebo",
                    "-s",
                    "libgazebo_ros_factory.so",
                    "-g",
                    "libpolygon_parser.so",
                     "--verbose",
                    "/workspace/rover/ros2/src/simulator/kiwi_gazebo/worlds/3_LMU.world",
                ],
                #output="screen",                
            ),         
            Node(
                package="spawning_services",
                executable="client", 
                #output="screen",                           
            ),
            Node(
                package="user_interface",
                executable="interface",
                output = "screen"
            ),
        ]
    )
