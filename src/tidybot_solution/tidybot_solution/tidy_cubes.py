#!/usr/bin/env python

# This script is expanded from the colour_contours.py script provided, accessible at below link:
# https://github.com/LCAS/teaching/blob/ee333a18b1717a8da0e8a981e216995fe1699fcb/cmp3103m_ros2_code_fragments/cmp3103m_ros2_code_fragments/colour_contours.py

# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from tidybot_interfaces.msg import CubeContext
import tf2_ros
import sensor_msgs.msg
import geometry_msgs.msg
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images

import message_filters as mf
import numpy as np
import enum

class State(enum.Enum):
    # This state is when we can no longer find any cubes to push to the end
    TIDYING_COMPLETE = 0;
    # The state when we are actively pushing a cube
    # PUSHING_CUBE -> RETURNING_HOME when target reached
    PUSHING_CUBE     = 1;
    # When we are looking for a cube to push
    # SEARCHING_CUBE -> ALIGNING_CUBE when cube found
    # We have a state for turning left and one for turning right
    SEARCHING_CUBE_L = 2;
    SEARCHING_CUBE_R = 3;
    # When we are preparing to push a cube
    # ALIGNING_CUBE -> PUSHING_CUBE
    ALIGNING_CUBE    = 4;
    # When we've finished pushing a cube and want to return home
    # RETURNING_HOME -> SEARCHING_CUBE
    RETURNING_HOME   = 5;


class TidyCubes(Node):
    """
    A finite-state machine to tidy cubes
    """
    
    def __init__(self):
        # Init our ros2 node with a name
        super().__init__("tidy_cubes");

        # TF init code adapted from docs https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html
        # Setup our TF listener
        # We initialise a buffer to store TF frames in, and cache 5 seconds worth
        self.tf_buffer   = tf2_ros.buffer.Buffer(cache_time=tf2_ros.Duration(seconds=5));
        # We setup a listener to fetch TF frames
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, node=self);

        # Setup a publisher to send goal poses to
        self.goal_pub    = self.create_publisher(geometry_msgs.msg.PoseStamped, "/goal_pose");

        # We are a finite-state machine so we need to store what state we're in
        # Begin with the SEARCHING_CUBE state
        self.state       = State.SEARCHING_CUBE;
    
        # TSS code sample expanded from this doc: https://github.com/ros2/message_filters/blob/541d8a5009b14aaae4d9fe52e101273e428bb5d0/index.rst
        # Subscribe to the cube info and LiDAR but make sure we get the same topic at the same time
        tss = mf.TimeSynchronizer([
            # The cube info
            mf.Subscriber(self, CubeContext, "/cube_info"),
            # The LiDAR scanner
            mf.Subscriber(self, sensor_msgs.msg.LaserScan, "/scan")
        ], queue_size=2);

        # Register our callback
        tss.registerCallback(self.cube_callback);


    def update_state(self, new_state: State):
        """
        Updates the current state into the next one
        """
        return 0;


    def cube_callback(self, cubes_msg: CubeContext, scan: sensor_msgs.msg.LaserScan):
        """
        The function to act on cube info
        """

        # If we're finished, take no action
        if (self.state == State.TIDYING_COMPLETE): return;
        
        # Make a list of cubes
        cubes = [];
        
        for i in range(len(cubes_msg.is_green)):
            cubes.append({
                "isGreen": cubes_msg.is_green[i],
                "area"   : cubes_msg.area[i],
                "heading": cubes_msg.heading[i],
                "range"  : cubes_msg.range[i]
            });
        
        # If there is no new cube then we don't need to do anything
        if (len(cubes) <= 0): return;
            


def main(args=None):
    print('Starting colour_contours.py');

    rclpy.init(args=args);

    colour_contours = IdentifyCubes();

    rclpy.spin(colour_contours);

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_contours.destroy_node();
    rclpy.shutdown();


# If this script is being imported don't execute main()
# If it's being directly executed, do run main()
if __name__ == '__main__':
    main();
