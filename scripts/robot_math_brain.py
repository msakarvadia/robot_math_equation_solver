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
        self.cursor_msg = CursorLocate(cursor_loc=-1,image_width=0)


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
        Position robot facing the cursor location.
        """

        cmd = Twist()
        cmd.linear.x = 0

        diff_adj = -1
        cursor = self.cursor_msg

        rate = rospy.Rate(10)
        while cursor.cursor_loc == -1 or diff_adj > 5 :
            cursor = self.cursor_msg

            if cursor.cursor_loc != -1:
                diff_adj = (cursor.image_width / 2) - cursor.cursor_loc 
                cmd.angular.z = min(0.001 * diff_adj, 0.2)
            else:
                cmd.angular.z = 0.2

            self.movement_pub.publish(cmd)
            rate.sleep()
    

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
        Callback method for positioning the cursor.
        """

        self.cursor_locator = cursor 



if __name__ == "__main__":
    node = SystemCoordinator()
    node.run()