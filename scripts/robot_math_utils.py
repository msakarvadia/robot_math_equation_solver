#!/usr/bin/env python3

import math

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan


# Set lidar model
LIDAR = "LDS"


class LidarSampler:
    """
    Utility class which samples lidar scans from a given sweep, and returns two
    numpy arrays representing the distances recorded and corresponding angles
    in radians.
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
        
        return scan_data, angles



def interpolate_path(points_array, resolution):
    """
    Helper function to interpolate character paths. Array passed is a numpy 
    array and resolution is the length traveled along path before creating a new 
    point.
    """

    # Calculate vectors between points and their magnitudes
    diff_vecs = np.diff(points_array, axis=0)
    magnitudes = np.linalg.norm(diff_vecs, axis=1, keepdims=True)

    # Calculate number of steps between each point and single step vector
    num_steps = (magnitudes / resolution).astype(int)
    step_vec = diff_vecs * (resolution / magnitudes)

    # Create new points array to fill in
    interpolated_array = np.empty([points_array.shape[0] + np.sum(num_steps), 
                                   points_array.shape[1]])

    # Fill in the points
    pos = 0
    for i, start_point in enumerate(points_array):
        interpolated_array[pos] = start_point
        pos += 1
        cum_step_point = np.zeros([1, points_array.shape[1]])
        if i < len(points_array) - 1:
            # Step from start point
            for _ in range(num_steps[i][0]):
                cum_step_point += step_vec[i]
                interpolated_array[pos] = start_point + cum_step_point
                pos += 1
    
    return interpolated_array 


def y_rotate_hack(char_path, rad):
    """
    Helper function hack to rotate character path about the y axis, when wall is
    not completely vertical.
    """
    rotation = np.array([[math.cos(rad), 0, -1 * math.sin(rad), 0],
                        [0, 1, 0, 0],
                        [math.sin(rad), 0, math.cos(rad), 0],
                        [0, 0, 0, 0]])

    rotated_points = np.dot(rotation, np.transpose(char_path)).transpose()
    
    return rotated_points
