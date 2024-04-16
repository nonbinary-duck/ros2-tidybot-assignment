import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

#
# Launch file based on the substitutions tutorial: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html
#

def generate_launch_description():

    # Line of code adapted from the nav2_bringup slam_launch.py file
    tidybot_dir = get_package_share_directory('tidybot_solution');
    
    # Initialise a launch description object to add to
    ld = LaunchDescription();

    ld.add_action(
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
        )
    );

    ld.add_action(
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
        )
    );

    # Also launch rviz with our config
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                os.path.join( tidybot_dir, "rviz", "tidybot.rviz" )
            ]
        )
    );

    # Spawn cubes
    ld.add_action(
        # Spawn some green cubes
        Node(
            package="uol_tidybot",
            executable="generate_objects"
        )
    );

    ld.add_action(
        # Spawn some red cubes
        #ros2 run uol_tidybot generate_objects --ros-args -p red:=true -p n_objects:=10
        Node(
            package="uol_tidybot",
            executable="generate_objects",
            ros_arguments=[
                "-p",
                "red:=true",
                "-p",
                "m_objects:=10"
            ]
        )
    );

    # Return our launch description we've generated
    return ld;

