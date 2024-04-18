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
        # Setup a publisher for our cube info topic
        self.cube_pub    = self.create_publisher(CubeContext, "/cube_info", 10);
        
        # Setup some "constants"
        # Relevant TF frames
        self.TF_FRAME_WORLD   = "map";
        self.TF_FRAME_CAM     = "depth_camera_link";
        # Value to add to colour for bounding box of banner
        self.BANNER_MODIFIER  = (0,0,128);
        self.LOWER_LIGHTNESS  = 64;
        self.UPPER_LIGHTNESS  = 255;
        self.LOWER_SATURATION = 150;
        self.UPPER_SATURATION = 255;
        # Define ranges for our colour recognition
        #   The ranges are broad to accommodate the real world
        # Define red ranges
        self.LOWER_RED_LOWER = np.array((0,   self.LOWER_SATURATION, self.LOWER_LIGHTNESS));
        # 14° (10) appears just in to orange to my eye & monitor
        self.UPPER_RED_LOWER = np.array((10,  self.UPPER_SATURATION, self.UPPER_LIGHTNESS));

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
        # The distance away after which a cube is considered sorted
        self.CUBE_SORTED_DIST = 0.90;
    
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


    def process_mask(self, mask: cv2.typing.MatLike, cv_image: cv2.typing.MatLike, isGreen: bool, depth_img: cv2.typing.MatLike):
        """
        Process the provided mask and overlay data to cv_image, which python hopefully passes by reference
        """

        # Initialise a list to write cube data to
        cubes = [];

        # In case the depth camera is a different resolution to the colour sensor
        # get a ratio between the two so we can scale pixel values
        depth_ratio = np.divide(depth_img.shape, cv_image.shape[0:2]);
        
        # Find contours in our mask
        contours, hierarchy = cv2.findContours(
            # Since OpenCV 3.2 findContours doesn't modify the source image so we don't need .copy()
            mask,
            # Return a list hierarchy instead of a tree, since we're not interested in the hierarchy
            cv2.RETR_LIST,
            # Simplify the contour to not return redundant information
            cv2.CHAIN_APPROX_SIMPLE);
        
        # Iterate over the contours we found in the segmented image
        for c in contours:
            # This allows to compute the area (in pixels) of a contour
            area = cv2.contourArea(c);
            
            # Only operate on contours of a reasonable area
            if area > 200.0:

                # The bounding box and moments initialisation is taken from the OpenCV Python documentation
                # https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html

                # Get the bounding box of this contour
                bnd_x, bnd_y, bnd_w, bnd_h = cv2.boundingRect(c);
                # Get the centroid of the bounding box (which is approximately the centroid of the cube)
                bnd_centroid       = np.array( ( bnd_x + (bnd_w * 0.5), bnd_y + (bnd_h * 0.5) ) ).astype(int);
                
                # We use the depth ratio in case our pixels aren't exact
                # We also cast in to integer so we can use it as an index
                bnd_centroid_depth = np.multiply(bnd_centroid, depth_ratio).astype(int);
                
                # Check if the centre of that rectangle is above or below the centre of the image
                isCube             = bnd_centroid[1] > (cv_image.shape[0] * 0.5);

                # Get the distance to the cube
                dist               = depth_img[bnd_centroid_depth[1], bnd_centroid_depth[0]];

                # We now want to also figure out what (approximate) heading our cube has
                # Normalise around the centre of the screen
                # This could be improved by correcting for distortion, but approximations are ok for our application
                heading            = ((bnd_centroid[0] / cv_image.shape[1]) * self.CAMERA_FOV) - (self.CAMERA_FOV * 0.5);
                
                # Draw the (simplified) outline of our contour in black
                cv2.drawContours(cv_image, contours=c, contourIdx=-1, color=(255, 0, 255), thickness=5);

                # Draw a bounding box for our contour
                cv2.rectangle(
                    cv_image,
                    pt1=(bnd_x, bnd_y),
                    pt2=(bnd_x + bnd_w, bnd_y + bnd_h),
                    color= (0,0,0) if (not isCube) else ((0,255,0) if isGreen else (0,0,255)),
                    thickness=2
                );

                # Draw the centroid as a thin circle
                cv2.circle(
                    cv_image,
                    center=bnd_centroid,
                    radius=5,
                    color=(255,255,255),
                    thickness=1
                );

                # Figure out if the cube has been sorted
                # This works because we always return to the centre of the arena
                isSorted = dist > self.CUBE_SORTED_DIST;

                if (isCube and (not isSorted)):
                    # Print data into the image at the origin of the bounding box
                    cv2.putText(cv_image, f"d: {dist:.2f}, h: {heading*360:.2f}", org=(bnd_x, bnd_y), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1.25, color= (0, 0, 0), thickness=2);
                    
                    # Store data about our cube
                    # Only if it's not been sorted
                    cubes.append({
                        "isGreen": isGreen,
                        "area"   : area,
                        "heading": heading,
                        "dist"   : dist
                    });
                elif (isCube):
                    # Print data into the image at the origin of the bounding box
                    # Tell the user that this cube is considered sorted
                    cv2.putText(cv_image, f"Cube could be sorted!", org=(bnd_x, bnd_y), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=1.25, color= (0, 255, 0), thickness=2);
                else:
                    # If it's the area marker, mark it on the overlay
                    if (isGreen):
                        cv2.putText(cv_image, "Green Area Marker", org=(bnd_x, bnd_y), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=2.0, color= (0, 255, 0), thickness=2);
                    else:
                        cv2.putText(cv_image, "Red Area Marker", org=(bnd_x, bnd_y), fontFace=cv2.FONT_HERSHEY_PLAIN, fontScale=2.0, color= (0, 0, 255), thickness=2);
        
        # Return our list of cubes we found
        return cubes;



    def camera_callback(self, cam_data: sensor_msgs.msg.Image, depth_data: sensor_msgs.msg.Image):
        """
        The function to act on camera data.

        Requires both depth and colour information simultaneously
        """

        # Attempt to get TF
        try:
            # Get the transform link between the camera and the world
            self.cam2world = self.tf_buffer.lookup_transform(target_frame=self.TF_FRAME_CAM, source_frame=self.TF_FRAME_WORLD, time=rclpy.time.Time());
        except tf2_ros.TransformException as ex:
            # Log and ignore if we cannot get this transform
            print(f"WARN: Could get camera->world ( {self.TF_FRAME_CAM} -> {self.TF_FRAME_WORLD} ) transform. {ex}");
            return;

        
        try:
            # Get the ros2 topic data and use the CvBridge instance
            # to cast it into an OpenCV image
            cv_image = self.br.imgmsg_to_cv2(cam_data, "bgr8");
            # We also want to fetch our depth image, which uses 1-channel
            # 32-bit floating point pixels (which is distance)
            depth_image = self.br.imgmsg_to_cv2(depth_data, "32FC1");

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

        # Process both the red and green thresholds
        # Get back the list of cubes
        greenCubes = self.process_mask(green_thresh, cv_image, isGreen=True, depth_img=depth_image);
        redCubes   = self.process_mask(red_thresh, cv_image, isGreen=False, depth_img=depth_image);

        # Initialise a cube message to send
        cube_state = CubeContext();
        # Take the header from the depth sensor as that is what's relevant to us
        cube_state.header = depth_data.header;
        # Initialise lists
        cube_state.is_green = [];
        cube_state.area     = [];
        cube_state.heading  = [];
        cube_state.range    = [];

        # Combine the lists of cubes and iterate over them to add into the cube_state
        for cube in (greenCubes + redCubes):
            # Append this cube to the state
            cube_state.is_green.append(cube["isGreen"]);
            cube_state.area.append(int(cube["area"]));
            cube_state.heading.append(cube["heading"]);
            cube_state.range.append(cube["dist"]);

        self.cube_pub.publish(cube_state);

        # Reduce the image size we render using imshow
        # Overwrite existing variable for memory usage
        cv_image = cv2.resize(cv_image, (0,0), fx=0.75, fy=0.75);
        cv2.imshow("Cube View", cv_image);
        
        # Occupy (block) the window between callbacks so it stays up
        cv2.waitKey(1);

def main(args=None):
    print('Starting identify_cubes.py');

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
