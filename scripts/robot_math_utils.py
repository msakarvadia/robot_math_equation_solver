#!/usr/bin/env python3

import math

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from skimage.measure import ransac, LineModelND


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


def calc_wall_projection():
    """
    Utility function that estimates the transformation matrix for projecting characters
    (or any other points from the robot's coordinate frame) onto a (flat, vertical) 
    wall in front of the robot. It first samples the points and then uses RANSAC 
    to estimate the wall's location.
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
    dist_to_wall = np.mean(wall_scan_dists)
    translation = np.array([dist_to_wall, 0, 0])

    # Build and flatten the matrix
    transform = np.zeros((4, 4))
    transform[:2, :2] = rotation
    transform[:3, 3] = translation
    transform[2, 2] = 1
    transform[3, 3] = 1

    return transform.flatten(), dist_to_wall


def y_rotate_hack(char_path, rad):
    """
    Utility function used when wall is not completely vertical. It rotates
    points on the wall about the y axis (the base of the wall).
    """

    rotation = np.array([[math.cos(rad), 0, -1 * math.sin(rad), 0],
                        [0, 1, 0, 0],
                        [math.sin(rad), 0, math.cos(rad), 0],
                        [0, 0, 0, 0]])

    rotated_points = np.dot(rotation, np.transpose(char_path)).transpose()
    
    return rotated_points


def interpolate_path(points_array, resolution):
    """
    Utility function which interpolates paths. Array passed is a numpy array and 
    resolution is the length traveled along path before creating a new point.
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
