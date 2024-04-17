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
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images

import message_filters as mf
import numpy as np
import cv2


class IdentifyCubes(Node):
    def __init__(self):
        # Init our ros2 node with a name
        super().__init__("identify_cubes");

        # Initialise the OpenCV-ROS2 bridge
        self.br = CvBridge();
        # Setup a window to display stuff in
        cv2.namedWindow("Cube View", 1);

        # TF init code adapted from docs https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html
        # Setup our TF listener
        # We initialise a buffer to store TF frames in, and cache 5 seconds worth
        self.tf_buffer   = tf2_ros.buffer.Buffer(cache_time=tf2_ros.Duration(seconds=5));
        # We setup a listener to fetch TF frames
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, node=self);
        self.cube_pub    = self.create_publisher(CubeContext, "/cube_info", 10);
        
        # Setup some "constants"
        # Relevant TF frames
        self.TF_FRAME_WORLD   = "map"
        self.TF_FRAME_CAM     = "depth_camera_link"
        # Value to add to colour for bounding box of banner
        self.BANNER_MODIFIER  = (0,0,128);
        self.LOWER_LIGHTNESS  = 20;
        self.UPPER_LIGHTNESS  = 255;
        self.LOWER_SATURATION = 150;
        self.UPPER_SATURATION = 255;
        # Define ranges for our colour recognition
        #   The ranges are broad to accommodate the real world
        # Define red ranges
        self.LOWER_RED_LOWER = np.array((0,   self.LOWER_SATURATION, self.LOWER_LIGHTNESS));
        # 28° (20) appears just in to orange to my eye & monitor
        self.UPPER_RED_LOWER = np.array((20,  self.UPPER_SATURATION, self.UPPER_LIGHTNESS));

        # Define the red ranges at the top of the scale
        # 330° (235) appears just in to pink to my eye & monitor
        self.LOWER_RED_UPPER = np.array((235, self.LOWER_SATURATION, self.LOWER_LIGHTNESS));
        self.UPPER_RED_UPPER = np.array((255, self.UPPER_SATURATION, self.UPPER_LIGHTNESS));

        # Define green values
        # 76° looks like lime
        self.LOWER_GREEN      = np.array((54,  self.LOWER_SATURATION, self.LOWER_LIGHTNESS));
        # 150° looks like teal
        self.UPPER_GREEN      = np.array((107, self.UPPER_SATURATION, self.UPPER_LIGHTNESS));
        # The camera's FoV in tau radians
        # It's 80° and 1 tau radians is a full circle so it's an easy conversion that way
        # tau = 2pi
        self.CAMERA_FOV       = 80.0/360.0;
    
        # TSS code sample expanded from this doc: https://github.com/ros2/message_filters/blob/541d8a5009b14aaae4d9fe52e101273e428bb5d0/index.rst
        # Subscribe to the depth camera and the colour camera
        tss = mf.TimeSynchronizer([
            # The colour sensor output
            mf.Subscriber(self, sensor_msgs.msg.Image, "/limo/depth_camera_link/image_raw"),
            # The depth sensor output
            mf.Subscriber(self, sensor_msgs.msg.Image, "/limo/depth_camera_link/depth/image_raw")
        ], queue_size=2);

        # Register our callback
        tss.registerCallback(self.camera_callback);



    def cube_callback(self, cubes: CubeContext, scan: sensor_msgs.msg.LaserScan):
        """
        The function to act on camera data.

        Requires both depth and colour information simultaneously
        """


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
