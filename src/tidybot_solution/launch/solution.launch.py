import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

#
# Launch file based on the substitutions tutorial: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html
#

def generate_launch_description():
    # Initialise a launch description object to add to
    ld = LaunchDescription();

    ld.add_action(
        # Launch our nodes
        Node(
            package="tidybot_solution",
            executable="identify_cubes"
        )
    );

    ld.add_action(
        # Launch our nodes
        Node(
            package="tidybot_solution",
            executable="tidy_cubes"
        )
    );

    ld.add_action(
        # Adjust the rate of the camera as specifying it through params file doesn't work
        ExecuteProcess(
            cmd=[[
                "ros2 param set ",
                "/limo/gazebo_ros_depth_camera_sensor ",
                "update_rate ",
                "10.0"
            ]],
            shell=True
        )
    );

    return ld;