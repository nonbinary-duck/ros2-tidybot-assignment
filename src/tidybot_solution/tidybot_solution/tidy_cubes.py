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
    # (alternatively) PUSHING_CUBE -> RETURNING_HOME if the robot gets lost (this sometimes happens)
    PUSHING_CUBE     = 1;
    # When we are looking for a cube to push
    # SEARCHING_CUBE -> ALIGNING_CUBE when cube found
    # SEARCHING_CUBE -> TIDYING_COMPLETE when no cube found
    # We have a state for turning left and one for turning right
    SEARCHING_CUBE = 2;
    # When we are preparing to push a cube
    # ALIGNING_CUBE -> PUSHING_CUBE
    # (alternatively) ALIGNING_CUBE -> SEARCHING_CUBE if the cube gets lost (this should never happen)
    ALIGNING_CUBE    = 4;
    # Wait for a bit
    # WAIT_AFTER_HOME -> SEARCHING_CUBE
    WAIT_AFTER_HOME = 5;
    # When we've finished pushing a cube and want to return home
    # RETURNING_HOME -> SEARCHING_CUBE
    RETURNING_HOME   = 6;
    # The state to begin in
    # START_STATE -> RETURNING_HOME
    START_STATE      = 7;


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
        self.vel_pub         = self.create_publisher(geometry_msgs.msg.Twist, "/cmd_vel", 10);

        # We are a finite-state machine so we need to store what state we're in
        # Begin with the SEARCHING_CUBE state
        self.state           = State.START_STATE;
        # We also use the navigation stack for movement since it works quite well,
        # and we need to know when we've concluded the goal it last set
        self.waiting_for_nav = False;
        self.goals_sent      = 0;
        # The time elapsed since we started searching or waiting
        self.time_elapsed    = 0.0;
        # Variables to make sure we don't get lost pushing
        self.last_pos        = [0.0,0.0];
        self.time_since_move = 0.0;

        # Setup some "constants"
        # Relevant TF frames
        self.TF_FRAME_WORLD          = "map";
        self.TF_FRAME_CAM            = "depth_camera_link";
        # Always send n goals
        self.SEND_N_GOALS            = 5;
        # Search for at most 12 seconds then conclude there are no cubes
        self.MAX_SEARCH_TIME         = 12.0;
        # We receive cubes at exactly 10Hz
        self.CUBE_FREQUENCY          = 10.0;
        # Acceptable heading in tau radians
        self.ACCEPTABLE_HEADING      = 5.0/360.0;
        # Smallest angular velocity
        self.BASE_ANGULAR_VEL        = 0.2;
        self.DIST_TO_WALL            = 1.5;
        self.ACCEPTABLE_DIST_TO_WALL = 1.1;
        # The time after which from the pushing state to move into the return home state
        self.MAX_NO_MOVEMENT_TIME    = 5.0;
        # What counts as having moved
        self.MOVEMENT_THRESHOLD      = 0.05;
        # Time to wait before searching
        self.WAIT_TIME               = 12.5;

        # Register our callback for cube info
        self.create_subscription(CubeContext, "/cube_info", self.cube_callback, 10);


    def send_goal(self, pos: typing.List[float], rotation: typing.List[float], world_space: bool = True):
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

    def action_state(self, chosen_cube=None):
        if   (self.state == State.SEARCHING_CUBE):
            # Setup a velocity to angle toward the cube
            vel = geometry_msgs.msg.Twist();
            vel.linear.x = 0.0;
            vel.linear.y = 0.0;
            vel.linear.z = 0.0;
            
            # Get if we should be turning left or right
            # Go faster than the alignment
            vel.angular.z = self.BASE_ANGULAR_VEL * 5;
            vel.angular.x = 0.0; vel.angular.y = 0.0;

            # Publish this velocity
            self.vel_pub.publish(vel);
        
        elif (self.state == State.RETURNING_HOME):
            self.send_goal([0.0,0.0,0.0], [0.0, 0.0, 0.0]);
        
        elif (self.state == State.ALIGNING_CUBE):
            # # If we're aligning with a cube, we obviously need that cube
            assert(chosen_cube != None);
            
            # Setup a velocity to angle toward the cube
            vel = geometry_msgs.msg.Twist();
            vel.linear.x = 0.0;
            vel.linear.y = 0.0;
            vel.linear.z = 0.0;
            
            # Get if we should be turning left or right
            vel.angular.z = (self.BASE_ANGULAR_VEL * -1) if (chosen_cube["heading"] > 0) else self.BASE_ANGULAR_VEL;
            vel.angular.x = 0.0; vel.angular.y = 0.0;

            # Publish this velocity
            self.vel_pub.publish(vel);
        
        elif (self.state == State.PUSHING_CUBE):
            # Get the pos of our robot
            pos        = self.cam2world.transform.translation;
            # Get the yaw of our robot and put in the range of 0->tau instead of -pi -> +pi
            yaw        = tft.euler_from_quaternion( [ self.cam2world.transform.rotation.x, self.cam2world.transform.rotation.y, self.cam2world.transform.rotation.z, self.cam2world.transform.rotation.w ] )[2] + np.pi;
            target_pos = [0.0,0.0,0.0];

            # Figure out what wall to go to
            if   (yaw > (0.25 * np.pi) and yaw < (0.75 * np.pi)):
                target_pos = [0.0, self.DIST_TO_WALL, 0.0];
                # Get the other coordinate for our target pos
                target_pos[0] = 1.3 * ((yaw - (0.25 * np.pi)) / (0.5 * np.pi));
                # Make sure it's on the correct size
                if (yaw < (0.5 * np.pi)): target_pos[0] *= -1;
            
            elif (yaw > (0.75 * np.pi) and yaw < (1.25 * np.pi)):
                target_pos = [self.DIST_TO_WALL, 0.0, 0.0];
            
                # Get the other coordinate for our target pos
                target_pos[1] = 1.3 * ((yaw - (0.75 * np.pi)) / (0.5 * np.pi));
                # Make sure it's on the correct size
                if (yaw < (0.5 * np.pi)): target_pos[1] *= -1;
            
            elif (yaw > (1.25 * np.pi) and yaw < (1.75 * np.pi)):
                target_pos = [0.0, -1.3, 0.0];
            
                # Get the other coordinate for our target pos
                target_pos[0] = 1.3 * ((yaw - (0.75 * np.pi)) / (0.5 * np.pi));
                # Make sure it's on the correct size
                if (yaw < (0.5 * np.pi)): target_pos[0] *= -1;
            else:
                target_pos = [-self.DIST_TO_WALL, 0.0, 0.0];

                # Get the other coordinate for our target pos
                # For this function the yaw is either between 1.75 pi and 2 pi
                # or between 0 pi and 0.25 pi
                if (yaw > (1.75 * np.pi)):
                    target_pos[1] = -1.3 * ((yaw - (1.75 * np.pi)) / (0.25 * np.pi));
                else:
                    target_pos[1] =  1.3 * (yaw / (0.25 * np.pi));
            
            
            # Push forward
            self.send_goal(target_pos, [0.0, 0.0, yaw]);

    def transition_state(self, new_state: State):
        """
        Updates the current state into the next one
        """

        # Log the change
        self.get_logger().info( f"STATE_CHANGE: Transitioning from {self.state} into {new_state}");

        # Update the state
        self.state = new_state;

        # Set our counter back to zero (then add one because we've sent a goal)
        self.goals_sent = 0;
    
        # Reset time elapsed searching
        self.time_elapsed = 0.0;
        # Reset movement info
        self.last_pos        = [0.0,0.0];
        self.time_since_move = 0.0;
    
    def send_many_state(self, chosen_cube = None):
        # Always make sure we action the state n times
        if (self.goals_sent <= self.SEND_N_GOALS):
            # Send a goal
            self.action_state(chosen_cube);
            # Increment our counter
            self.goals_sent += 1;
    
    def select_cube(self, cubes):
        """
        Select a cube according to heading and range preference
        """
        
        selected_cube = cubes[0];
        selected_preference = 100000;
        
        # Find a cube
        for cube in cubes:
            # Get a preference factor
            # Prefer closer cubes in a similar heading to us
            preference = (np.abs(cube["heading"]) * 20) + cube["range"];
            # If our preference is better, update
            if (preference < selected_preference):
                selected_cube       = cube;
                selected_preference = preference;
    
        # Return our selected cube
        return selected_cube;


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


        # Check if we're supposed to be waiting
        if (self.state == State.WAIT_AFTER_HOME):

            if (self.time_elapsed > self.WAIT_TIME):
                # If we've finished the waiting period, send many state
                self.transition_state(State.SEARCHING_CUBE);
                self.send_many_state();
                # Exit
                return;

            # If we still are waiting, record that
            self.time_elapsed += np.power(self.CUBE_FREQUENCY, -1);
            # Do nothing else
            return;


        # If we're finished, take no action
        # It would be nice if python added switches
        if (self.state == State.TIDYING_COMPLETE): return;
            
    
        # If we've just started, then we first "return home"
        if (self.state == State.START_STATE):
            self.transition_state(State.RETURNING_HOME);
            # Send this state
            self.send_many_state();
        # If we're returning home, check if we're there
        elif (self.state == State.RETURNING_HOME):
            # Make sure we've sent enough states
            self.send_many_state();
        
            # If we're close to home, then go to the next state
            pos_x = np.abs(self.cam2world.transform.translation.x);
            pos_y = np.abs(self.cam2world.transform.translation.y);
            if (pos_x < 0.2 and pos_y < 0.2):
                # Go to the next state and send it
                self.transition_state(State.WAIT_AFTER_HOME);
            
            return;
        elif (self.state == State.PUSHING_CUBE):
            # Make sure we've sent enough states
            self.send_many_state();

            if (self.time_since_move > self.MAX_NO_MOVEMENT_TIME):
                # If we've not moved in a while return home
                self.transition_state(State.RETURNING_HOME);
                self.send_many_state();
                # Do not continue from here
                return;
            
            # Check if we've moved
            this_pos = [ self.cam2world.transform.translation.x, self.cam2world.transform.translation.y ];
            dist_moved = np.abs(self.last_pos[0] - this_pos[0]) + np.abs(self.last_pos[1] - this_pos[1])
            if (dist_moved >= self.MOVEMENT_THRESHOLD):
                # Reset time since we last moved
                self.time_since_move = 0.0;
                # Reset the last pos if over the threshold to make sure slow movement doesn't break us
                self.last_pos = this_pos;
            else:
                # Increment time since last move
                self.time_since_move += np.power(self.CUBE_FREQUENCY, -1);
            
            # Get distance from centre
            pos_y = np.abs(self.cam2world.transform.translation.y);
            pos_x = np.abs(self.cam2world.transform.translation.x);
            # If we've reached a wall then we can return to home state
            if (pos_y > self.ACCEPTABLE_DIST_TO_WALL or pos_x > self.ACCEPTABLE_DIST_TO_WALL):
                self.transition_state(State.RETURNING_HOME);
                # Send this state
                self.send_many_state();
                
        # Make a list of cubes
        cubes = [];
        
        # Compress our multiple lists into one for convenience
        for i in range(len(cubes_msg.is_green)):
            cubes.append({
                "isGreen": cubes_msg.is_green[i],
                "area"   : cubes_msg.area[i],
                "heading": cubes_msg.heading[i],
                "range"  : cubes_msg.range[i]
            });
        
        # If there is no new cube
        if (len(cubes) <= 0):
            if (self.state == State.SEARCHING_CUBE):
                # Check if we've exceeded the time allocated for searching cubes
                if (self.time_elapsed > self.MAX_SEARCH_TIME):
                    self.transition_state(State.TIDYING_COMPLETE);
                else:
                    # If we haven't then increment the time elapsed
                    self.time_elapsed += np.power(self.CUBE_FREQUENCY, -1);
                    # And continue searching
                    self.action_state();
            # This state should never happen, but just in case it does...
            elif (self.state == State.ALIGNING_CUBE):
                self.transition_state(State.SEARCHING_CUBE);
                self.send_many_state();
            
            # Do not continue / no need to continue
            return;
    
        # If we've found a cube and we're searching for one, pick one and change state
        if (self.state == State.SEARCHING_CUBE):
            # Pick a cube
            selected_cube = self.select_cube(cubes);

            # Update our state
            self.transition_state(State.ALIGNING_CUBE);
            # We don't care about the counter here since we deal with this velocity
            self.action_state(selected_cube);
        
        elif (self.state == State.ALIGNING_CUBE):
            # Make the aligning process closed-loop
            # Pick the same cube again (or a better one if found)
            selected_cube = self.select_cube(cubes);

            # If we're within an acceptable range of the cube, then change state
            if (np.abs(selected_cube["heading"]) < self.ACCEPTABLE_HEADING):
                # Change state
                self.transition_state(State.PUSHING_CUBE);
                self.send_many_state();
            else:
                # Otherwise, continue the alignment
                self.action_state(selected_cube);


            
        
            


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
