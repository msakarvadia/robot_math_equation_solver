#!/usr/bin/env python3

import math

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from skimage.measure import ransac, LineModelND
from std_msgs.msg import Header
from tf import TransformListener
from tf.transformations import euler_from_quaternion

from robot_math_equation_solver.srv import Cursor, CursorResponse
from robot_math_equation_solver.msg import CursorLocate
from robot_math_utils import LidarSampler


# Set debug mode
DEBUG = True

# Constants for cursor positioning and size
START_HEIGHT = 0.37
CURSOR_WIDTH = 0.025
CURSOR_HEIGHT = 0.05

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


class CursorWallTransformation:
    """
    A service which tracks the current location of the "virtual wall cursor".

    Upon making a "GET" request, clients will receive the current cursor 
    location and associated transformation matrix to write 2D character paths onto
    the wall.

    Upon a "NEXT" request, the service will advance the cursor to the next 
    character and respond with this position and associated matrix.
    """

    def __init__(self):
        rospy.init_node("robot_math_wall_cursor_server")

        # Start cursor position subscriber
        self.cursor_locator = rospy.Subscriber("/robot_math/cursor_position", CursorLocate, self.cursor_position_callback)

        # Cursor data
        self.y_offset = None
        self.z_offset = START_HEIGHT
        self.new_line_y_offset = None
        self.cur_char_pos = 0
        self.transform = []
        self.angle_to_cursor = None

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
        odom_pose = self.get_cur_pose()
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


    def get_cur_pose(self):
        """
        Sample a single lidar scan and get current pose
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
        Resets the cursor based on location published by the /robot_math/cursor_locator 
        topic.
        """

        self.calc_wall_transformation()

        # Wait for initialization
        rate = rospy.Rate(10)
        while self.angle_to_cursor is None:
            rate.sleep()

        self.new_line_y_offset = math.tan(self.angle_to_cursor) * self.dist_to_wall
        self.y_offset = self.new_line_y_offset


    def calc_wall_transformation(self):
        """
        Method which estimates the transformation matrix for writing characters
        onto a (flat, vertical) wall in front of the robot. It first samples the 
        points and then uses RANSAC to estimate the walls location.
        """

        # Draw 10 lidar scans to use in estimation
        wall_points = np.empty([0,2]) 
        wall_scan_dists = np.empty([0,])
        for _ in range(10):
            # Convert readings to cartesian coordinates and stack the points
            scan_data, angles = LidarSampler.lidar_front()
            x = np.cos(angles) * scan_data
            y = np.sin(angles) * scan_data
            lidar_sample = np.column_stack((x, y))
            wall_points = np.concatenate((wall_points, lidar_sample))
            wall_scan_dists = np.concatenate((wall_scan_dists, scan_data))

        # Estimate wall location
        model, _ = ransac(wall_points, 
                          LineModelND, 
                          min_samples=5,
                          max_trials=100,
                          residual_threshold=0.1)
        _, direction_vector = model.params

        # Calculate rotation
        if direction_vector[1] / direction_vector[0] < 0:
            rot = 1
        else:
            rot = -1
        rot_vector = abs(direction_vector)
        theta = rot * math.atan2(rot_vector[0], rot_vector[1])
        rotation = np.array([[math.cos(theta), -1 * math.sin(theta)],
                             [math.sin(theta), math.cos(theta)]])

        # Calculate translation
        self.dist_to_wall = np.mean(wall_scan_dists)
        translation = np.array([self.dist_to_wall, 0, 0])

        # Build and flatten the matrix
        transform = np.zeros((4, 4))
        transform[:2, :2] = rotation
        transform[:3, 3] = translation
        transform[2, 2] = 1
        transform[3, 3] = 1

        self.transform = transform.flatten()


    def cursor_position_callback(self, cursor):
        """
        Callback method which updates the angle to the cursor based on the
        cursor position publisher.
        """

        # Calculate hand-eye angle mapping
        if DEBUG == True:
            self.angle_to_cursor = 0.25
        else:
            angle_from_left_edge = ((abs(CAMERA_ANGLE_L) + abs(CAMERA_ANGLE_R)) 
                                    * cursor.cursor_loc / cursor.image_width)
            if angle_from_left_edge > 0:
                self.angle_to_cursor = CAMERA_ANGLE_L - angle_from_left_edge


if __name__ == "__main__":
    server = CursorWallTransformation()
    rospy.spin()
