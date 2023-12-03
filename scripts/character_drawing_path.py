#!/usr/bin/env python3

import math
from collections import deque
from json import load
from os.path import dirname

import numpy as np

import rospy
from std_msgs.msg import String

from robot_math_equation_solver.srv import CharacterPath, CharacterPathResponse, Cursor


# Scaling and offset configuration
PATH_RESOLUTION = 0.005                     # interpolation along path
CHARACTER_SCALING = 0.002                   # characters set to 0.04m wide
# PEN_OFFSET = 0.006                        # the pen is about 0.07m long
PEN_OFFSET = -0.015                         # the pen is about 0.07m long
PEN_LIFT = {                                # pen lift offsets
    "x": -0.05,
    "y": -1 * 10 * CHARACTER_SCALING, 
    "z": 10 * CHARACTER_SCALING,
}


def interpolate_path(points_array, resolution):
    """
    Helper function to linearly interpolate character paths. Array passed is a 
    numpy array and resolution is the length traveled along path before creating
    a new point.
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
    not completely vertical
    """
    rotation = np.array([[math.cos(rad), 0, -1 * math.sin(rad), 0],
                        [0, 1, 0, 0],
                        [math.sin(rad), 0, math.cos(rad), 0],
                        [0, 0, 0, 0]])

    rotated_points = np.dot(rotation, np.transpose(char_path)).transpose()
    
    return rotated_points


class CharacterPathGenerator:
    """
    A ROS service which provides 3D trajectories for writing characters onto an
    identified wall (using the "virtual cursor" service) in front of the robot.

    Upon making a request message with the boolean True, a client will receive 
    the complete trajectory of the next segment (character). These 3D points can
    then be input to the inverse kinematic model.

    The service uses a vectorized Hershey font with a 20x21 frame to describe 
    each character.
    """

    def __init__(self):
        rospy.init_node("robot_math_character_path_server")

        # Start service
        self.segment_service = rospy.Service("/robot_math/character_path_service", 
                                             CharacterPath, 
                                             self.service_response)

        # Subscribe to computer vision node and cursor service
        self.math_string_sub = rospy.Subscriber("/robot_math/math_strings", 
                                                String, 
                                                self.math_string_callback)
        self.advance_cursor = rospy.ServiceProxy("/robot_math/wall_cursor_service", 
                                                 Cursor)

        # Load vectorized characters dictionary
        with open(dirname(dirname(__file__))+"/resources/hershey_font.json") as f:
            self.characters = load(f)

        # Initialize service queue
        self.segment_queue = deque()

 
    def service_response(self, request):
        """
        Request callback method for the service.
        """

        # Respond with next enqueued path, if it exists
        if len(self.segment_queue) == 0:
            return CharacterPathResponse(point_path=[], dim=0)
        elif request.request:
            return CharacterPathResponse(point_path=self.segment_queue.popleft(), dim=3)


    def math_string_callback(self, math_string):
        """
        Callback method which monitors the /robot_math/math_strings topic where
        the complete strings are published.
        """

        # Process each character in string
        for char in math_string.data:

            # Create 4D base character path
            base_path = self.get_base_path(char)

            # If invalid character
            if base_path.size == 0:
                continue

            # Get current cursor location
            cursor_loc = self.advance_cursor_client()
            if cursor_loc is None:
                continue

            # Add the cursor offset and calculate wall transformation
            #char_path = y_rotate_hack(base_path, -.1)
            char_path = base_path
            char_path += np.array([0, cursor_loc.y_offset, cursor_loc.z_offset, 1])
            transform = np.array(cursor_loc.transform).reshape(4, 4)
            char_path = np.dot(transform, np.transpose(char_path)).transpose()

            # Enqueue flattened list of 3D points for inverse kinematics
            self.segment_queue.append(list(char_path[:,:3].flatten()))


    def get_base_path(self, char):
        """
        Helper method which creates a 4D base character path from the 2D hershey
        font. This path array will then be projected onto the wall.
        """

        # Initialize new array of points
        char_path = np.empty([])

        if self.characters.get(char) is not None:
            # Get flattened 2D character path points array
            flattened_char_2d = self.characters[char][1]

            # Initialize 4D point array
            char_path = np.zeros((len(flattened_char_2d) // 2, 4))

            # Populate the array and scale the movements
            for i, coord in enumerate(flattened_char_2d):

                if i % 2 == 0:
                    if coord == -1:
                        char_path[i//2][0] = PEN_OFFSET + PEN_LIFT["x"]
                        char_path[i//2][1] = PEN_LIFT["y"]
                    else:
                        char_path[i//2][0] = PEN_OFFSET
                        char_path[i//2][1] = -1 * coord * CHARACTER_SCALING

                else:
                    if coord == -1:
                        char_path[(i-1)//2][2] = PEN_LIFT["z"]
                    else:
                        char_path[(i-1)//2][2] = coord * CHARACTER_SCALING

        # Return an interpolated path
        return interpolate_path(char_path, PATH_RESOLUTION)


    def advance_cursor_client(self):
        """
        Cursor service client method which advances the virtual cursor and
        retrieves its updated location along with the associated transformation 
        matrix.
        """

        rospy.wait_for_service("/robot_math/wall_cursor_service")

        try:
            resp = self.advance_cursor(request="NEXT")
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


if __name__ == "__main__":
    server = CharacterPathGenerator()
    rospy.spin()