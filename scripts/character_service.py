#!/usr/bin/env python3

import json
from os.path import dirname
from collections import deque

import rospy

from std_msgs.msg import String
from robot_math_equation_solver.srv import CharacterPath, CharacterPathResponse


CHARACTER_SCALING = 0.002  # Characters set to 0.04m wide
DIST_TO_BOARD = 0.3


class CharacterService:

    def __init__(self):

        rospy.init_node("robot_math_character_path_server")

        # Start service
        self.segment_service = rospy.Service("/robot_math/character_path_service", CharacterPath, self.service_response)
 
        # Subscribe to computer vision node
        self.math_string_sub = rospy.Subscriber("/robot_math/math_strings", String, self.math_string_callback)

        # Load vectorized characters dictionary
        with open(dirname(dirname(__file__))+"/resources/hershey_font.json") as f:
            self.characters = json.load(f)

        # Initialize service queue
        self.segment_queue = deque()

 
    def service_response(self, request):

        # Process valid requests
        if request.request and len(self.segment_queue) > 0:
            return CharacterPathResponse(point_path=self.segment_queue.popleft(), dim=3)


    def math_string_callback(self, math_string):

        # Set where the pen should go when lifted
        pen_lift_y = 10 * CHARACTER_SCALING
        pen_lift_z = 0.03

        # Process each character in string
        for char in math_string.data:
            if self.characters.get(char) is not None:

                # Get flattened character path points array
                char_template = self.characters[char][1]

                # Initialize new array of points
                char_path = []

                # Populate the char_path array and scale the movements
                for i, coord in enumerate(char_template):
                    if i % 2 == 0:
                        char_path.append(DIST_TO_BOARD)
                        if coord == -1:
                            char_path.append(pen_lift_y)
                        else:
                            char_path.append(coord * CHARACTER_SCALING)
                    else:
                        if coord == -1:
                            char_path.append(pen_lift_z)
                        else:
                            char_path.append(coord * CHARACTER_SCALING)

                self.segment_queue.append(char_path)


if __name__ == "__main__":
    server = CharacterService()
    rospy.spin()
