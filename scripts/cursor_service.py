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
LINE_WIDTH = 0.04


def sample_lidar_in_front():
    scan_data = rospy.wait_for_message("/scan", LaserScan, timeout=1)
    # Return scan data from +- Pi/2 in front
    if LIDAR == "LDS":
        return scan_data.ranges[315:360] + scan_data.ranges[0:46]
    elif LIDAR == "RP":
        return scan_data.ranges[430:717]


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
        self.min_dist_to_wall = None

        # Cursor reset
        self.cursor_set = False


    def reset_cursor(self):
        self.calc_wall_transformation()

        # Wait for initialization
        rate = rospy.Rate(10)
        while self.angle_to_cursor is None:
            rate.sleep()

        self.y_offset = math.tan(self.angle_to_cursor) * self.min_dist_to_wall


    def calc_wall_transformation(self):

        # Estimate wall in robot's coordinate frame using a lidar reading
        wall_lidar_distances = sample_lidar_in_front()
        print(wall_lidar_distances)
        model, _ = ransac(wall_lidar_distances, LineModelND, min_samples=2, max_trials=50, residual_threshold=0.1)
        intercept, line_vector_est = model.params

        # Initialize transformation matrix and the required values
        transform = np.zeros((4, 4))
        theta = math.atan2(line_vector_est[1], line_vector_est[0])
        self.min_dist_to_wall = intercept[1] / math.sqrt((line_vector_est[1] / line_vector_est[0])**2 + 1)

        # Calculate rotation and translation
        rotation = np.array([[math.cos(theta), -1 * math.sin(theta)],[math.sin(theta), math.cos(theta)]])
        translation = np.array([self.min_dist_to_wall, 0, 0])

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
            self.y_offset += LINE_WIDTH

        return CursorResponse(y_offset=self.y_offset,
                              z_offset=self.z_offset,
                              transform=self.transform)


    def cursor_locator_callback(self, data):
        self.angle_to_cursor = data.image_width - data.cursor_loc 


if __name__ == "__main__":
    server = WallCursorService()
    rospy.spin()