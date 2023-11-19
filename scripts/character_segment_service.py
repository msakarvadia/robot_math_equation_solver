#!/usr/bin/env python3

import json
from os.path import dirname
from collections import deque

import rospy

from std_msgs.msg import String
from robot_math_equation_solver.srv import SegmentGen, SegmentGenResponse


CHARACTER_SCALING = 0.002  # Characters set to 0.04m wide
DIST_TO_BOARD = 0.3


class SegmentService:

    def __init__(self):
        rospy.init_node("robot_math_segment_server")

        # Start service
        self.segment_service = rospy.Service("/robot_math/segment_generator", SegmentGen, self.segment_service_response)
 
        # Subscribe to computer vision node
        self.math_string_sub = rospy.Subscriber("/robot_math/math_strings", String, self.math_string_callback)

        # Load vectorized characters dictionary
        with open(dirname(dirname(__file__))+"/resources/hershey_font.json") as f:
            self.characters = json.load(f)

        # Initialize service queue
        self.segment_queue = deque()
    
 
    def segment_service_response(self, request):

        # Process valid requests
        if request.request and len(self.segment_queue) > 0:
            return SegmentGenResponse(segment=self.segment_queue.popleft(), cols=3)


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

                # Populate the char_path array and scale the movments
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


    def run(self):
        rospy.spin()


if __name__ == "__main__":
    server = SegmentService()
    server.run()
