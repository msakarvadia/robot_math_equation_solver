#!/usr/bin/env python3

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