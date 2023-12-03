#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


from nn import process_and_predict_answer_from_cropped_images
from robot_math_utils import LidarSampler


class SystemCoordinator:
    """
    """


    def __init__(self):
        rospy.init_node("robot_math_brain")

        # Subscribe to computer vision node and cursor service
        self.math_string_pub = rospy.Publisher("/robot_math/math_strings", String, queue_size=10, latch=True)
     

    def run(self):

        # Run the math solving process
        while not rospy.is_shutdown():
            self.find_cursor()

            self.move_to_viewing_pos()

            math_string = process_and_predict_answer_from_cropped_images()

            self.move_to_board()

            self.math_string_pub(math_string)

            self.move_to_viewing_pos()

            rospy.sleep(20)


    def find_cursor(self):
        pass
    
    def move_to_board(self):
        pass

    def move_to_viewing_pos(self):
        pass


if __name__ == "__main__":
    node = SystemCoordinator()
    node.run()