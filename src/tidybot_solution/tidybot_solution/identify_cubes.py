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


class ColourContours(Node):
    def __init__(self):
        super().__init__('colour_contours')
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)

        self.br = CvBridge()

    def camera_callback(self, data):
        #self.get_logger().info("camera_callback")

        # Setup a window to display stuff in
        cv2.namedWindow("Cube View", 1)

        try:
            # Get the ros2 topic data and use the CvBridge instance
            # to cast it into an OpenCV image
            cv_image = self.br.imgmsg_to_cv2(data, "bgr8")

        # Ignore errors by printing them and stopping the execution 
        # only of this callback
        except CvBridgeError as e:
            print(e)
            return


        # It often is better to use another colour space, that is
        # less sensitive to illumination (brightness) changes.
        # The HSV colour space is often a good choice. 
        # So, we first change the colour space here...
        # HSV is also good because the hue differentiates
        # between red/green for example
        # 
        # Convert our image to the HSV colour space
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # ... and now let's create a binary (mask) image, looking for 
        # any hue (range: 0-255), but for something brightly
        # colours (high saturation: > 150)
        hsv_thresh = cv2.inRange(hsv_img,
                                 np.array((0, 150, 50)),
                                 np.array((255, 255, 255)))

        # just for the fun of it, print the mean value 
        # of each HSV channel within the mask 
        print("Mean of all HSV channels in mask:")
        print(cv2.mean(hsv_img[:, :, 0], mask = hsv_thresh)[0])
        print(cv2.mean(hsv_img[:, :, 1], mask = hsv_thresh)[0])
        print(cv2.mean(hsv_img[:, :, 2], mask = hsv_thresh)[0])

        # Find contours in our mask
        contours, hierachy = cv2.findContours(
            # Since OpenCV 3.2 findContours doesn't modify the source image so we don't need .copy()
            hsv_thresh,
            # Return a tree hierarchy instead of a list, so that obviously related contours can form a tree
            cv2.RETR_TREE,
            # Simplify the contour to not return redundant information
            cv2.CHAIN_APPROX_SIMPLE)
        
        # Iterate over the contours we found in the segmented image
        for c in contours:
            # This allows to compute the area (in pixels) of a contour
            area = cv2.contourArea(c)
            
            # Only operate on contours of a specified area
            if area > 100.0:
                # Draw the (simplified) outline of our contour
                cv2.drawContours(cv_image, c, -1, (255, 0, 255), 10)
                # Print data into the image at the first vertex in the contour
                cv2.putText(cv_image, f"{area}", org=c[0][0], fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=2.0, color= (0, 0, 0), thickness=2)
        print('====')

        cv_image_small = cv2.resize(cv_image, (0,0), fx=0.75, fy=0.75) # reduce image size
        cv2.imshow("Cube View", cv_image_small)
        
        cv2.waitKey(1)

def main(args=None):
    print('Starting colour_contours.py.')

    rclpy.init(args=args)

    colour_contours = ColourContours()

    rclpy.spin(colour_contours)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_contours.destroy_node()
    rclpy.shutdown()


# If this script is being imported don't execute main()
# If it's being directly executed, do run main()
if __name__ == '__main__':
    main()
