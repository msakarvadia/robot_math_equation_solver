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

    # Radians-to-index mappings
    lds = np.fromfunction(lambda k: k * np.pi / (180), (360,))
    rp = np.fromfunction(lambda k: ((573.5 + k) % 1147) * np.pi / (573.5), (1147,))

    @classmethod
    def lidar_front(cls):
        # Sample a single lidar scan 
        data = None
        while data is None:
            data = rospy.wait_for_message("/scan", LaserScan, timeout=1)
        ranges = np.array(data.ranges)

        # Adjust for different lidars and only sample from -Pi/4 to Pi/4
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
       
        # Convert to cartesian coordinates and stack into (x. y) points
        x = np.cos(angles) * scan_data
        y = np.sin(angles) * scan_data

        return np.column_stack((x, y))


class WallCursorService:

    def __init__(self):

        rospy.init_node("robot_math_wall_cursor_server")

        # Start service
        self.segment_service = rospy.Service("/robot_math/wall_cursor_service", Cursor, self.service_response)

        # Start cursor locator subscriber
        self.cursor_locator = rospy.Subscriber("/robot_math/cursor_locator", CursorLocate, self.cursor_locator_callback)

        # Cursor data
        self.y_offset = None
        self.z_offset = START_HEIGHT
        self.transform = []
        self.angle_to_cursor = None
        self.dist_to_wall = None

        # Cursor reset
        self.cursor_set = False


    def reset_cursor(self):
        self.calc_wall_transformation()

        # Wait for initialization
        rate = rospy.Rate(10)
        while self.angle_to_cursor is None:
            rate.sleep()

        self.y_offset = math.tan(self.angle_to_cursor) * self.dist_to_wall


    def calc_wall_transformation(self):

        # Estimate wall in robot's coordinate frame using a lidar reading
        wall_points = SampleWallPoints.lidar_front()
        model, _ = ransac(wall_points, LineModelND, min_samples=2, max_trials=100, residual_threshold=0.1)
        origin, direction_vector = model.params

        # Initialize transformation matrix and the required values
        transform = np.zeros((4, 4))
        theta = math.atan2(-1 * direction_vector[0], direction_vector[1])
        intercept = (direction_vector[1] / direction_vector[0]) * (-1 * origin[0]) + origin[1] 
        self.dist_to_wall = abs(intercept) / math.sqrt((direction_vector[1] / direction_vector[0])**2 + 1)

        # Calculate rotation and translation
        rotation = np.array([[math.cos(theta), -1 * math.sin(theta)],[math.sin(theta), math.cos(theta)]])
        translation = np.array([self.dist_to_wall, 0, 0])

        # Build the matrix
        transform[:2, :2] = rotation
        transform[:3, 3] = translation
        transform[2, 2] = 1
        transform[3, 3] = 1

        self.transform = transform.flatten()


    def service_response(self, r):

        # Set position on first cursor request or after a movement
        if not self.cursor_set:
            self.reset_cursor()
            self.cursor_set == True

        # Handle request 
        if r.request == "NEXT":
            self.y_offset += CHAR_WIDTH

        return CursorResponse(y_offset=self.y_offset,
                              z_offset=self.z_offset,
                              transform=self.transform)


    def cursor_locator_callback(self, data):
        # TODO: Get camera angles
        self.angle_to_cursor = data.image_width - data.cursor_loc 


if __name__ == "__main__":
    server = WallCursorService()
    rospy.spin()