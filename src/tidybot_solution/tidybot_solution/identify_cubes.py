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
        # Write a new variable so we don't have to convert back to
        # RGB later on for human viewing
        cv_image_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV);

        # Create a mask for green
        green_thresh = cv2.inRange(cv_image_hsv, lowerb=self.LOWER_GREEN, upperb=self.UPPER_GREEN);


        # Create a binary mask for the lower red
        red_thresh = cv2.inRange(cv_image_hsv, lowerb=self.LOWER_RED_LOWER, upperb=self.UPPER_RED_LOWER);
        # Combine that image with the upper red threshold
        red_thresh = cv2.bitwise_or(
            red_thresh,
            # Our upper red threshold
            cv2.inRange(cv_image_hsv, lowerb=self.LOWER_RED_UPPER, upperb=self.UPPER_RED_UPPER)
        );

        # Find contours in our mask
        contours, hierachy = cv2.findContours(
            # Since OpenCV 3.2 findContours doesn't modify the source image so we don't need .copy()
            green_thresh,
            # Return a list hierarchy instead of a tree, since we're not interested in the hierarchy
            cv2.RETR_LIST,
            # Simplify the contour to not return redundant information
            cv2.CHAIN_APPROX_SIMPLE);
        
        # Iterate over the contours we found in the segmented image
        for c in contours:
            # This allows to compute the area (in pixels) of a contour
            area = cv2.contourArea(c);
            
            # Only operate on contours of a specified area
            if area > 100.0:

                # The bounding box and moments initialisation is taken from the OpenCV Python documentation
                # https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html

                # Get the bounding box of this contour
                bnd_x, bnd_y, bnd_w, bnd_h = cv2.boundingRect(c);
                # Check if the centre of that rectangle is above or below the centre of the image
                isCube  = (bnd_y + (bnd_h * 0.5)) > (cv_image.shape[0] * 0.5);
                # Get the aspect ratio of the cube, to figure out if it is actually one cube
                ratio   = bnd_w / bnd_h;
                
                # Draw the (simplified) outline of our contour in black
                cv2.drawContours(cv_image, contours=c, contourIdx=-1, color=(255, 0, 255), thickness=5);

                # Draw a bounding box for our contour
                cv2.rectangle(
                    cv_image,
                    pt1=(bnd_x, bnd_y),
                    pt2=(bnd_x + bnd_w, bnd_y + bnd_h),
                    color= (0,255,0) if isCube else (0,0,0),
                    thickness=2
                );

                if (isCube):
                    # Print data into the image at the origin of the bounding box
                    cv2.putText(cv_image, f"a: {area:.0f}, r: {ratio:.3f}", org=(bnd_x, bnd_y), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1.25, color= (0, 0, 0), thickness=2);
                else:
                    cv2.putText(cv_image, "Green Area Marker", org=(bnd_x, bnd_y), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=2.0, color= (0, 255, 0), thickness=2);
                    
                

        # Reduce the image size we render using imshow
        # Overwrite existing variable for memory usage
        cv_image = cv2.resize(cv_image, (0,0), fx=0.75, fy=0.75);
        cv2.imshow("Cube View", cv_image);
        
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
