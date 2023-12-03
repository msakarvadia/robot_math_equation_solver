#!/usr/bin/env python3

import math

import numpy as np
from skimage.measure import ransac, LineModelND

import rospy
from sensor_msgs.msg import LaserScan

from robot_math_equation_solver.srv import Cursor, CursorResponse
from robot_math_equation_solver.msg import CursorLocate


# Set debug mode
DEBUG = True

# Set lidar model
LIDAR = "LDS"

# Set height of cursor and character width
START_HEIGHT = 0.37
CHAR_WIDTH = 0.025
CHAR_HEIGHT = 0.05

# Set hand-eye calibration for manipulator-camera operation
CAMERA_ANGLE_L = 0.2792
CAMERA_ANGLE_R = -0.1919


class SampleWallPoints:
    """
    Utility class which samples lidar scans from a given sweep, and converts
    the data points to cartesian frame of the robot.
    """

    # Lidar radians-to-index mappings (precomputed)
    lds = np.fromfunction(lambda k: k * np.pi / (180), (360,))
    rp = np.fromfunction(lambda k: ((573.5 + k) % 1147) * np.pi / (573.5), (1147,))

    @classmethod
    def lidar_front(cls):
        """
        Class method which samples from the sector -Pi/8 to Pi/8.
        """

        # Sample a single lidar scan 
        data = None
        while data is None:
            data = rospy.wait_for_message("/scan", LaserScan, timeout=1)
        ranges = np.array(data.ranges)

        # Adjust for lidar differences
        if LIDAR == "LDS":
            scan_left = np.array([ranges[i] for i in range(337, 360)])
            scan_right = np.array([ranges[i] for i in range(0, 23)])
            scan_data = np.concatenate((scan_left, scan_right), axis=0)

            angles_left = np.array([cls.lds[i] for i in range(337, 360)])
            angles_right = np.array([cls.lds[i] for i in range(0, 23)])
            angles = np.concatenate((angles_left, angles_right), axis=0)

        elif LIDAR == "RP":
            scan_data = np.array([ranges[i] for i in range(502, 646)])
            angles = np.array([cls.rp[i] for i in range(502, 646)])
 
        # Convert coordinates and stack into (x. y) points
        x = np.cos(angles) * scan_data
        y = np.sin(angles) * scan_data

        return np.column_stack((x, y))


class WallCursorService:
    """
    A ROS service which tracks the current location of the "virtual wall cursor".

    Upon making a "GET" request, clients will receive the current cursor 
    location and associated transformation matrix to write 2D character paths onto
    the wall.

    Upon a "NEXT" request, the service will advance the cursor to the next 
    character and respond with this position and associated matrix.
    """

    def __init__(self):
        rospy.init_node("robot_math_wall_cursor_server")

        # Start service
        self.segment_service = rospy.Service("/robot_math/wall_cursor_service", 
                                             Cursor, 
                                             self.service_response)

        # Start cursor locator subscriber
        self.cursor_locator = rospy.Subscriber("/robot_math/cursor_locator", 
                                               CursorLocate, 
                                               self.cursor_locator_callback)

        # Cursor data
        self.y_offset = None
        self.z_offset = START_HEIGHT
        self.new_line_y_offset = None
        self.cur_char_pos = 0
        self.transform = []
        self.angle_to_cursor = None
        self.dist_to_wall = None

        # Cursor reset
        self.cursor_set = False


    def service_response(self, r):
        """
        Request callback method for service.
        """

        # Set position on first cursor request or after a movement
        if not self.cursor_set:
            self.reset_cursor()
            self.cursor_set = True

        # Handle requests
        elif r.request == "NEXT":
            # Write 5 characters per line
            if self.cur_char_pos == 5:
                self.y_offset = self.new_line_y_offset
                self.z_offset -= CHAR_HEIGHT
                self.cur_char_pos = 0
            else:
                self.y_offset -= CHAR_WIDTH

        self.cur_char_pos += 1

        return CursorResponse(y_offset=self.y_offset,
                              z_offset=self.z_offset,
                              transform=self.transform)


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
        wall_points = np.empty([1,2]) 
        for _ in range(10):
            wall_points = np.concatenate((wall_points, SampleWallPoints.lidar_front()))

        print(len(wall_points))
        # Estimate wall location
        model, _ = ransac(wall_points, 
                          LineModelND, 
                          min_samples=5,
                          max_trials=100,
                          residual_threshold=0.1)

        origin, direction_vector = model.params

        # Calculate rotation
        theta = math.atan2(direction_vector[0], direction_vector[1])
        rotation = np.array([[math.cos(theta), -1 * math.sin(theta)],
                             [math.sin(theta), math.cos(theta)]])

        # Calculate translation
        c = ((direction_vector[1] / direction_vector[0]) * (-1 * origin[0]) + origin[1])
        self.dist_to_wall = (abs(c) / math.sqrt((direction_vector[1] / direction_vector[0])**2 + 1))
        translation = np.array([self.dist_to_wall, 0, 0])

        # Build and flatten the matrix
        transform = np.zeros((4, 4))
        transform[:2, :2] = rotation
        transform[:3, 3] = translation
        transform[2, 2] = 1
        transform[3, 3] = 1

        self.transform = transform.flatten()


    def cursor_locator_callback(self, img):
        """
        Callback method which updates the angle to the cursor based on the
        computer vision node tracking it.
        """

        # Calculate hand-eye angle mapping
        if DEBUG == True:
            #self.angle_to_cursor = 0.25
            self.angle_to_cursor = 0.0
        else:
            angle_from_left_edge = ((abs(CAMERA_ANGLE_L) + abs(CAMERA_ANGLE_R)) 
                                    * img.cursor_loc / img.image_width)
            if angle_from_left_edge != 0:
                self.angle_to_cursor = CAMERA_ANGLE_L - angle_from_left_edge


if __name__ == "__main__":
    server = WallCursorService()
    rospy.spin()
