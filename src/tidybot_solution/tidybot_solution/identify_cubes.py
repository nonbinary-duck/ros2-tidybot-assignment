#!/usr/bin/env python

# This script is expanded from the colour_contours.py script provided, accessible at below link:
# https://github.com/LCAS/teaching/blob/ee333a18b1717a8da0e8a981e216995fe1699fcb/cmp3103m_ros2_code_fragments/cmp3103m_ros2_code_fragments/colour_contours.py

# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np




class IdentifyCubes(Node):
    def __init__(self):
        # Init our ros2 node with a name
        super().__init__("identify_cubes");
        # Subscribe to the depth camera
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1);
        # Initialise the OpenCV-ROS2 bridge
        self.br = CvBridge();
        # Setup a window to display stuff in
        cv2.namedWindow("Cube View", 1);
        # Setup some "constants"
        cv2.LOWER_LIGHTNESS  = 20;
        cv2.UPPER_LIGHTNESS  = 255;
        cv2.LOWER_SATURATION = 150;
        cv2.UPPER_SATURATION = 255;


    def camera_callback(self, data):
        try:
            # Get the ros2 topic data and use the CvBridge instance
            # to cast it into an OpenCV image
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8");

        # Ignore errors by printing them and stopping the execution 
        # only of this callback
        except CvBridgeError as e:
            print(e);
            return;


        # It often is better to use another colour space, that is
        # less sensitive to illumination (brightness) changes.
        # The HSV colour space is often a good choice. 
        # There is an argument for using YCbCr as there isn't two
        # ways to make red, an important colour for us, but in YCbCr
        # there is a direct boundary between green and red which
        # could cause issues in the real world where colour may not
        # be correctly reported, or affected by ambient diffusion
        # 
        # Convert our image to the HSV colour space
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV);

        # Create a binary mask for red and green
        red_thresh = cv2.inRange(cv_image, lowerb=np.array((0, 150, 50)), upperb=np.array((255, 255, 255)));
        # 
        red_thresh = cv2.bitwise_or();

        # Find contours in our mask
        contours, hierachy = cv2.findContours(
            # Since OpenCV 3.2 findContours doesn't modify the source image so we don't need .copy()
            hsv_thresh,
            # Return a tree hierarchy instead of a list, so that obviously related contours can form a tree
            cv2.RETR_TREE,
            # Simplify the contour to not return redundant information
            cv2.CHAIN_APPROX_SIMPLE);
        
        # Iterate over the contours we found in the segmented image
        for c in contours:
            # This allows to compute the area (in pixels) of a contour
            area = cv2.contourArea(c);
            
            # Only operate on contours of a specified area
            if area > 100.0:
                # Draw the (simplified) outline of our contour
                cv2.drawContours(cv_image, c, -1, (255, 0, 255), 10);
                # Print data into the image at the first vertex in the contour
                cv2.putText(cv_image, f"{area}", org=c[0][0], fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=2.0, color= (0, 0, 0), thickness=2);
        print('====');

        # Reduce the image size we render using imshow
        cv_image_small = cv2.resize(cv_image, (0,0), fx=0.75, fy=0.75);
        cv2.imshow("Cube View", cv_image_small);
        
        cv2.waitKey(1);

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
