#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from tf import TransformListener
from tf.transformations import euler_from_quaternion

from robot_math_equation_solver.srv import Cursor, CursorResponse
from robot_math_equation_solver.msg import CursorLocate
from robot_math_utils import calc_wall_projection


# Set debug mode
DEBUG = True

# Constants for cursor positioning and size
START_HEIGHT = 0.35
CURSOR_WIDTH = 0.04
CURSOR_HEIGHT = 0.045

# Set calibration for manipulator-camera operation
CAMERA_ANGLE_L = 0.2792
CAMERA_ANGLE_R = -0.1919


def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw """

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw


class WallCursorTransformation:
    """
    A service which tracks the current location of the "virtual wall cursor".

    Upon making a "GET" request, clients will receive the current cursor 
    location and associated transformation matrix to write 2D character paths onto
    the wall.

    Upon a "NEXT" request, the service will advance the cursor to the next 
    character and respond with this position and associated matrix.
    """


    def __init__(self):
        rospy.init_node("robot_math_wall_cursor_transformation_server")

        # Start cursor position subscriber
        self.cursor_locator = rospy.Subscriber("/robot_math/tag_position", CursorLocate, self.tag_position_callback)

        # Cursor data
        self.y_offset = None
        self.z_offset = START_HEIGHT
        self.new_line_y_offset = None
        self.cur_char_pos = 0
        self.transform = []
        self.angle_to_cursor = 0.25

        self.dist_to_wall = None

        # Threshold values for linear and angular movement before resetting cursor
        self.lin_mvmt_threshold = 0.03
        self.ang_mvmt_threshold = (math.pi / 32)

        # Last motion update's pose
        self.odom_pose_last_motion_update = None
        self.tf_listener = TransformListener()

        # Cursor reset
        self.cursor_set = False

        # Start the service
        self.segment_service = rospy.Service("/robot_math/wall_cursor_service", Cursor, self.service_response)


    def service_response(self, r):
        """
        Request callback method for service.
        """

        # Check if robot moved
        odom_pose = self.get_cur_robot_pose()
        curr_x = odom_pose.pose.position.x
        old_x = self.odom_pose_last_motion_update.pose.position.x
        curr_y = odom_pose.pose.position.y
        old_y = self.odom_pose_last_motion_update.pose.position.y
        curr_yaw = get_yaw_from_pose(odom_pose.pose)
        old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

        # If robot has moved, reset wall cursor
        if (not self.cursor_set or 
            abs(curr_x - old_x) > self.lin_mvmt_threshold or 
            abs(curr_y - old_y) > self.lin_mvmt_threshold or
            abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):
            self.reset_cursor()
            self.cursor_set = True

        # Check if cursor being advanced
        if r.request == "NEXT":
            # Write 5 characters per line, after which advance to next line
            if self.cur_char_pos < 5:
                self.y_offset -= CURSOR_WIDTH
            elif self.cur_char_pos == 5:
                self.y_offset = self.new_line_y_offset
                self.z_offset -= CURSOR_HEIGHT
                self.cur_char_pos = 0

            self.cur_char_pos += 1

        return CursorResponse(y_offset=self.y_offset,
                              z_offset=self.z_offset,
                              transform=self.transform)


    def get_cur_robot_pose(self):
        """
        Get current robot pose from 'odom' topic.
        """

        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id="base_footprint"),
            pose=Pose())

        odom_pose = self.tf_listener.transformPose("/odom", p)

        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = odom_pose

        return odom_pose


    def reset_cursor(self):
        """
        Resets the cursor based on location published by the 'robot_math/cursor_locator' 
        topic.
        """

        # Reset projection to wall
        self.transform, self.dist_to_wall = calc_wall_projection()

        # Reset 'new_line' coordinate position on wall 
        self.new_line_y_offset = math.tan(self.angle_to_cursor) * self.dist_to_wall
        self.y_offset = self.new_line_y_offset


    def tag_position_callback(self, cursor):
        """
        Callback method which updates the angle to the cursor based on the
        tag position.
        """

        # Calculate hand-eye angle mapping
        if DEBUG == True:
            self.angle_to_cursor = 0.25 # Debug does not use tag position
        else:
            angle_from_left_edge = ((abs(CAMERA_ANGLE_L) + abs(CAMERA_ANGLE_R)) 
                                    * cursor.cursor_loc / cursor.image_width)
            if angle_from_left_edge > 0:
                self.angle_to_cursor = CAMERA_ANGLE_L - angle_from_left_edge


if __name__ == "__main__":
    server = WallCursorTransformation()
    rospy.spin()
