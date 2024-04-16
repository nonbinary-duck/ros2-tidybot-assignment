import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node

#
# Launch file based on the substitutions tutorial: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html
#

def generate_launch_description():

    # Line of code adapted from the nav2_bringup slam_launch.py file
    tidybot_dir = get_package_share_directory('tidybot_solution');
    
    return LaunchDescription([
        # Include the tidybot launch description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("uol_tidybot"),
                    "launch",
                    "tidybot.launch.py"
                ])
            ]),
            launch_arguments={
                "use_rviz": "false",
                "params_file": os.path.join( tidybot_dir, "params", "limo_params.yaml" )
            }.items()
        ),
        # Include our Nav2 SLAM Toolbox launch description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "slam_launch.py"
                ])
            ]),
            launch_arguments={
                "slam_params_file": os.path.join( tidybot_dir, "params", "limo_mapper_params_online_sync.yaml" ),
                "params_file": os.path.join( tidybot_dir, "params", "limo_nav2_params.yaml" )
            }.items()
        ),
        # Spawn some green cubes
        Node(
            package="uol_tidybot",
            executable="generate_objects",
        ),
        # Spawn some red cubes
        #ros2 run  uol_tidybot generate_objects --ros-args -p red:=true -p n_objects:=10
        Node(
            package="uol_tidybot",
            executable="generate_objects",
            ros_arguments=[
                "red:=true",
                "m_objects:=10"
            ]
        ),
        # Launch our nodes
        Node(
            package="tidybot_solution",
            executable="identify_cubes"
        ),
        # Adjust the rate of the camera as specifying it through params file doesn't work
        ExecuteProcess(
            cmd=[[
                "ros2 param set ",
                "/limo/gazebo_ros_depth_camera_sensor ",
                "update_rate ",
                "20.0"
            ]],
            shell=True
        )
    ]);