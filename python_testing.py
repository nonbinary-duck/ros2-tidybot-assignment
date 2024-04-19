import rclpy
from rclpy.node import Node

import tf2_ros
import geometry_msgs.msg
import tf_transformations as tft # Alias to tft because that's a big name
import nav2_simple_commander.robot_navigator as nav2sc # Alias because it's a long name


import message_filters as mf
import numpy as np
import typing
import enum

#navigator = nav2sc.BasicNavigator;

def send_goal(pos: typing.List[float], rotation: typing.List[float], navigator, world_space: bool = True):
    """
    Send a target pose to nav2

    Arguments:
        pos      -- The target position in Cartesian coordinates (x,y)
        rotation -- The target rotation in euler roll/pitch/yaw
    """

    # Get the quaternion rotation from the euler angles
    rot_quat = tft.quaternion_from_euler(rotation[0], rotation[1], rotation[2]);

    # Halt the navigator since we want to do this task now
    navigator.cancelTask();
    
    # Setup the message
    goal                    = geometry_msgs.msg.PoseStamped();
    # We use the same header information as our TF since that should be up-to-date
    # If we have a goal not for the world space, keep the frame relative to the robot
    if (world_space): goal.header.frame_id = "map";
    
    goal.pose.position.x    = pos[0];
    goal.pose.position.y    = pos[1];
    goal.pose.position.z    = pos[2];
    goal.pose.orientation.x = rot_quat[0];
    goal.pose.orientation.y = rot_quat[1];
    goal.pose.orientation.z = rot_quat[2];
    goal.pose.orientation.w = rot_quat[3];

    # Send our goal
    navigator.goToPose(goal);