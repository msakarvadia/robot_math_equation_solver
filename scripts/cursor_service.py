#!/usr/bin/env python3

import math

import numpy as np
from skimage.measure import ransac, LineModelND

import rospy
from sensor_msgs.msg import LaserScan

from robot_math_equation_solver.srv import Cursor, CursorResponse
from robot_math_equation_solver.msg import CursorLocate


LIDAR = "LDS"
START_HEIGHT = 0.19
CHAR_WIDTH = 0.04


class SampleWallPoints:
    """
    Utility class which samples lidar scans from a given sweep, and converts
    the data points to cartesian coordinate frame of lidar.
    """

    # Lidar radians-to-index mappings (precomputed)
    lds = np.fromfunction(lambda k: k * np.pi / (180), (360,))
    rp = np.fromfunction(lambda k: ((573.5 + k) % 1147) * np.pi / (573.5), (1147,))

    @classmethod
    def lidar_front(cls):
        """
        Class method which samples from the sector -Pi/4 to Pi/4.
        """

        # Sample a single lidar scan 
        data = None
        while data is None:
            data = rospy.wait_for_message("/scan", LaserScan, timeout=1)
        ranges = np.array(data.ranges)

        # Adjust for lidar differences
        if LIDAR == "LDS":
            scan_left = np.array([ranges[i] for i in range(315, 360)])
            scan_right = np.array([ranges[i] for i in range(0, 46)])
            scan_data = np.concatenate((scan_left, scan_right), axis=0)

            angles_left = np.array([cls.lds[i] for i in range(315, 360)])
            angles_right = np.array([cls.lds[i] for i in range(0, 46)])
            angles = np.concatenate((angles_left, angles_right), axis=0)

        elif LIDAR == "RP":
            scan_data = np.array([ranges[i] for i in range(430, 717)])
            angles = np.array([cls.rp[i] for i in range(430, 717)])
       
        # Convert coordinates and stack into (x. y) points
        x = np.cos(angles) * scan_data
        y = np.sin(angles) * scan_data

        return np.column_stack((x, y))


class WallCursorService:
    """
    A ROS service which tracks the current location of the "virtual wall cursor".

    Upon making a "GET" request, clients will receive the current cursor 
    location and associated transformation matrix that maps 2D character paths to
    to wall being written on.

    Upon making a "NEXT" request, the service will advance the cursor to the
    next character and respond with this position.
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
        self.transform = []
        self.angle_to_cursor = None
        self.dist_to_wall = None

        # Cursor reset
        self.cursor_set = False


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

        self.y_offset = math.tan(self.angle_to_cursor) * self.dist_to_wall


    def calc_wall_transformation(self):
        """
        Method which estimates the transformation matrix for mapping characters
        onto a (flat) wall in front of the robot. It first samples the points and
        then uses RANSAC to estimate the walls location.
        """

        wall_points = SampleWallPoints.lidar_front()

        # Estimate wall in robot's coordinate frame using RANSAC
        model, _ = ransac(wall_points, 
                          LineModelND, 
                          min_samples=2, 
                          max_trials=100, 
                          residual_threshold=0.1)

        origin, direction_vector = model.params

        # Initialize transformation matrix
        transform = np.zeros((4, 4))

        # Calculate rotation
        theta = math.atan2(-1 * direction_vector[0], direction_vector[1])
        rotation = np.array([[math.cos(theta), -1 * math.sin(theta)],
                             [math.sin(theta), math.cos(theta)]])

        # Calculate translation
        intercept = ((direction_vector[1] / direction_vector[0]) 
                     * (-1 * origin[0]) + origin[1])
        self.dist_to_wall = (abs(intercept) 
                             / math.sqrt((direction_vector[1] / direction_vector[0])**2 + 1))
        translation = np.array([self.dist_to_wall, 0, 0])

        # Build the matrix
        transform[:2, :2] = rotation
        transform[:3, 3] = translation
        transform[2, 2] = 1
        transform[3, 3] = 1

        self.transform = transform.flatten()


    def service_response(self, r):
        """
        Request callback method for service.
        """

        # Set position on first cursor request or after a movement
        if not self.cursor_set:
            self.reset_cursor()
            self.cursor_set == True

        # Handle requests
        elif r.request == "NEXT":
            self.y_offset += CHAR_WIDTH

        return CursorResponse(y_offset=self.y_offset,
                              z_offset=self.z_offset,
                              transform=self.transform)


    def cursor_locator_callback(self, img):
        """
        Callback method which updates the angle to the cursor based on the
        computer vision node tracking it.
        """

        # TODO: Get camera field of view angles
        self.angle_to_cursor = img.image_width - img.cursor_loc 


if __name__ == "__main__":
    server = WallCursorService()
    rospy.spin()