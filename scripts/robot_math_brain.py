#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from nn import process_and_predict_answer_from_cropped_images
from robot_math_equation_solver.msg import CursorLocate
from robot_math_utils import LidarSampler


class SystemCoordinator:
    """
    """


    def __init__(self):
        rospy.init_node("robot_math_brain")

        # Start robot movements publisher
        self.movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10, latch=True)

        # Start drawing movements publisher
        self.drawing_pub = rospy.Publisher("/robot_math/math_strings", String, queue_size=10, latch=True)
        
        # Start cursor locator subscriber
        self.cursor_locator = rospy.Subscriber("/robot_math/cursor_position", CursorLocate, self.cursor_position_callback) 

        self.cursor_msg = None


    def run(self):
        """
        """

        # Run the math solving process
        while not rospy.is_shutdown():
            self.find_cursor()

            self.move_to_viewing_pos()

            math_string = process_and_predict_answer_from_cropped_images()

            self.move_to_board()

            self.drawing_pub(math_string)

            self.move_to_viewing_pos()

            rospy.sleep(20)


    def find_cursor(self):
        """
        """

        # The tolerable range of the goal x-coordinate
        while pos == -1 or pos > middle + goal_range or pos < middle - goal_range:
            self.vel_twist.angular.z = 0.1
            pos = self.get_goal_pos(msg)
            
        self.vel_twist.angular.z = 0.0
    

    def move_to_board(self):
        """
        """

        pass


    def move_to_viewing_pos(self):
        """
        """

        pass

    def cursor_position_callback(self, cursor):
        """
        """

        
         




if __name__ == "__main__":
    node = SystemCoordinator()
    node.run()