#!/usr/bin/env python

# This script is expanded from the colour_contours.py script provided, accessible at below link:
# https://github.com/LCAS/teaching/blob/ee333a18b1717a8da0e8a981e216995fe1699fcb/cmp3103m_ros2_code_fragments/cmp3103m_ros2_code_fragments/colour_contours.py

# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

import tf_transformations as tft # Alias to tft because that's a big name
from tidybot_interfaces.msg import CubeContext
import tf2_ros
import sensor_msgs.msg
import geometry_msgs.msg
from cv_bridge import CvBridge, CvBridgeError # Package to convert between ROS and OpenCV Images

import message_filters as mf
import numpy as np
import typing
import enum

class State(enum.Enum):
    # This state is when we can no longer find any cubes to push to the end
    TIDYING_COMPLETE = 0;
    # The state when we are actively pushing a cube
    # PUSHING_CUBE -> RETURNING_HOME when target reached
    PUSHING_CUBE     = 1;
    # When we are looking for a cube to push
    # SEARCHING_CUBE -> ALIGNING_CUBE when cube found
    # SEARCHING_CUBE -> TIDYING_COMPLETE when no cube found
    # We have a state for turning left and one for turning right
    SEARCHING_CUBE_L = 2;
    SEARCHING_CUBE_R = 3;
    # When we are preparing to push a cube
    # ALIGNING_CUBE -> PUSHING_CUBE
    ALIGNING_CUBE    = 4;
    # When we've finished pushing a cube and want to return home
    # RETURNING_HOME -> SEARCHING_CUBE
    RETURNING_HOME   = 5;
    # The state to begin in
    # START_STATE -> RETURNING_HOME
    START_STATE      = 6;


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
        self.tf_buffer       = tf2_ros.buffer.Buffer(cache_time=tf2_ros.Duration(seconds=5));
        # We setup a listener to fetch TF frames
        self.tf_listener     = tf2_ros.transform_listener.TransformListener(self.tf_buffer, node=self);

        # Setup a publisher to send goal poses to
        self.goal_pub        = self.create_publisher(geometry_msgs.msg.PoseStamped, "/goal_pose", 10);

        # We are a finite-state machine so we need to store what state we're in
        # Begin with the SEARCHING_CUBE state
        self.state           = State.START_STATE;
        # We also use the navigation stack for movement since it works quite well,
        # and we need to know when we've concluded the goal it last set
        self.waiting_for_nav = False;

        # Setup some "constants"
        # Relevant TF frames
        self.TF_FRAME_WORLD   = "map";
        self.TF_FRAME_CAM     = "depth_camera_link";

        # Register our callback for cube info
        self.create_subscription(CubeContext, "/cube_info", self.cube_callback, 10);


    def send_goal(self, pos: typing.List[float], rotation: typing.List[float], world_space: bool = False):
        """
        Send a target pose to nav2

        Arguments:
            pos      -- The target position in Cartesian coordinates (x,y)
            rotation -- The target rotation in euler roll/pitch/yaw
        """

        # Get the quaternion rotation from the euler angles
        rot_quat = tft.quaternion_from_euler(rotation[0], rotation[1], rotation[2]);

        # Setup the message
        goal                  = geometry_msgs.msg.PoseStamped();
        # We use the same header information as our TF since that should be up-to-date
        goal.header           = self.cam2world.header;
        # If we have a goal not for the world space, keep the frame relative to the robot
        if (world_space): goal.header.frame_id = self.TF_FRAME_WORLD;
        
        goal.pose.position.x  = pos[0];
        goal.pose.position.y  = pos[1];
        goal.pose.position.z  = pos[2];
        goal.pose.orientation.x = rot_quat[0];
        goal.pose.orientation.y = rot_quat[1];
        goal.pose.orientation.z = rot_quat[2];
        goal.pose.orientation.w = rot_quat[3];
    
        # Send our goal
        self.goal_pub.publish(goal);

    def action_state(self):
        if   (self.state == State.SEARCHING_CUBE_L):
            self.send_goal([0.0,0.0,0.0], [0.0, 0.0, np.pi]);
        
        elif (self.state == State.SEARCHING_CUBE_R):
            self.send_goal([0.0,0.0,0.0], [0.0, 0.0, -np.pi]);
        
        elif (self.state == State.RETURNING_HOME):
            self.send_goal([1.0,0.0,0.0], [0.0, 0.0, 0.0], world_space=True);
        
        elif (self.state == State.ALIGNING_CUBE):
            # If we're aligning with a cube, we obviously need that cube
            assert(chosen_cube != None);
            # Get the rotation of our robot
            euler = tft.euler_from_quaternion( [ self.cam2world.transform.rotation.x, self.cam2world.transform.rotation.y, self.cam2world.transform.rotation.z, self.cam2world.transform.rotation.w ] );
            # Set our robot to align with a cube
            self.send_goal([0.0,0.0,0.0], [0.0, 0.0, euler[2] + (chosen_cube["heading"] * 2 * np.pi)]);
        
        elif (self.state == State.PUSHING_CUBE):
            # Get the pos of our robot
            pos        = self.cam2world.transform.translation;
            # Get the yaw of our robot
            yaw        = tft.euler_from_quaternion(self.cam2world.transform.rotation)[2] + np.pi;
            target_pos = [0.0,0.0,0.0];

            # Figure out what wall to go to
            if   (yaw > (0.25 * np.pi) and yaw < (0.75 * np.pi)):
                target_pos = [pos.x, 1.4, 0.0];
            elif (yaw > (0.75 * np.pi) and yaw < (1.25 * np.pi)):
                target_pos = [1.4, pos.y, 0.0];
            elif (yaw > (1.25 * np.pi) and yaw < (1.75 * np.pi)):
                target_pos = [pos.x, -1.4, 0.0];
            else:
                target_pos = [-1.4, pos.y, 0.0];
            
            # Push forward
            self.send_goal(target_pos, [0.0, 0.0, euler[2]]);

    def transition_state(self, new_state: State, chosen_cube = None):
        """
        Updates the current state into the next one, and execute associated actions
        """

        # Update the state
        self.state = new_state;
    
        # Take action on that state
        self.action_state();


    def cube_callback(self, cubes_msg: CubeContext):
        """
        The function to act on cube info
        """

        # Attempt to get TF
        try:
            # Get the transform link between the camera and the world
            self.cam2world = self.tf_buffer.lookup_transform(target_frame=self.TF_FRAME_CAM, source_frame=self.TF_FRAME_WORLD, time=rclpy.time.Time());
        except tf2_ros.TransformException as ex:
            # Log and ignore if we cannot get this transform
            print(f"WARN: Could get camera->world ( {self.TF_FRAME_CAM} -> {self.TF_FRAME_WORLD} ) transform. {ex}");
            return;

        # self.get_logger().info( f"yaw: {tft.euler_from_quaternion([ self.cam2world.transform.rotation.x, self.cam2world.transform.rotation.y, self.cam2world.transform.rotation.z, self.cam2world.transform.rotation.w ])[2] + np.pi}");
        # return;

        # # If we're waiting for the navigation stack to complete a task,
        # # then we have nothing to do here
        # if (self.waiting_for_nav): return;

        # If we're finished, take no action
        # It would be nice if python added switches
        if (self.state == State.TIDYING_COMPLETE): return;
        elif (self.state == State.SEARCHING_CUBE_L):
            pass;
    
        # If we've just started, then we first "return home"
        if (self.state == State.START_STATE):
            self.transition_state(State.RETURNING_HOME);
        
        self.action_state();
        
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
    print('Starting tidy_cubes.py');

    rclpy.init(args=args);

    controller = TidyCubes();

    rclpy.spin(controller);

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node();
    rclpy.shutdown();


# If this script is being imported don't execute main()
# If it's being directly executed, do run main()
if __name__ == '__main__':
    main();
