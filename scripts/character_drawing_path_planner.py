#!/usr/bin/env python3

import math
from collections import deque
from json import load
from os.path import dirname

import numpy as np
import rospy
from std_msgs.msg import String

from robot_math_equation_solver.srv import CharacterPath, CharacterPathResponse, Cursor
from robot_math_utils import interpolate_path, y_rotate_hack


LIDAR_OFFSET = 0.032                    # lidar is offset from manipulator frame
CHAR_PATH_RESOLUTION = 0.001            # interpolation resolution in m
CHAR_SCALE = 0.002                      # characters set to 0.04m wide (0.002 * 20)
PEN_LIFT = {                            # position when pen is lifted
    "x": -0.03,
    "y": -1 * 10 * CHAR_SCALE, 
    "z": 10 * CHAR_SCALE,
}


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

        # Start the service
        self.segment_service = rospy.Service("/robot_math/character_path_service", CharacterPath, self.service_response)

        # Start math string subscriber service
        self.math_string_sub = rospy.Subscriber("/robot_math/math_strings", String, self.math_string_callback)

        # Start cursor service proxy
        rospy.wait_for_service("/robot_math/wall_cursor_service")
        self.advance_cursor = rospy.ServiceProxy("/robot_math/wall_cursor_service", Cursor)

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
            cursor = self.get_cursor()
            if cursor is None:
                continue

            # Add the cursor offset and calculate wall transformation
            char_path = y_rotate_hack(base_path, 0.3)
            #char_path = base_path
            char_path += np.array([0, cursor.y_offset, cursor.z_offset, 1])
            transform = np.array(cursor.transform).reshape(4, 4)
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
            # Get flattened character path points
            flattened_char_2d = self.characters[char][1]

            # Initialize 4D point array
            char_path = np.zeros((len(flattened_char_2d) // 2, 4))

            # Populate 4D array with 2D character path adding pen lift dimension
            for i, coord in enumerate(flattened_char_2d):

                # Scale coordinates and check if pen lift (-1, -1)
                if i % 2 == 0:
                    # Set x and y
                    if coord == -1:
                        char_path[i//2][0] = LIDAR_OFFSET + PEN_LIFT["x"]
                        char_path[i//2][1] = PEN_LIFT["y"]
                    else:
                        char_path[i//2][0] = LIDAR_OFFSET
                        char_path[i//2][1] = -1 * coord * CHAR_SCALE

                else:
                    # Set z
                    if coord == -1:
                        char_path[(i-1)//2][2] = PEN_LIFT["z"]
                    else:
                        char_path[(i-1)//2][2] = coord * CHAR_SCALE

        # Return an interpolated path
        return interpolate_path(char_path, CHAR_PATH_RESOLUTION)


    def get_cursor(self):
        """
        Method which advances the cursor location and retrieves the associated 
        wall projection matrix.
        """

        try:
            resp = self.advance_cursor(request="NEXT")
            return resp

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


if __name__ == "__main__":
    server = CharacterPathGenerator()
    rospy.spin()