#!/usr/bin/env python3

import numpy as np

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from nn import process_and_predict_answer_from_cropped_images
from robot_math_equation_solver.msg import CursorLocate
from robot_math_utils import LidarSampler


TARGET_BOARD_DIST = 0.3
TARGET_VIEWING_DIST = 0.8


class RobotMathControlNode:
    """
    """


    def __init__(self):
        rospy.init_node("robot_math_master_control")

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

            self.reposition("REVERSE")

            math_string = process_and_predict_answer_from_cropped_images()

            self.reposition("APPROACH")

            self.drawing_pub(math_string)

            self.reposition("REVERSE")

            rospy.sleep(20)


    def find_cursor(self):
        """
        Rotate robot until facing the cursor location.
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
    

    def reposition(self, command):
        """
        Move the robot to and from the drawing and viewing positions.
        """
        if command == "APPROACH":
            direction = 1 
        elif command == "REVERSE":
            direction = -1

        cmd = Twist()
        front_avg_dist = 999

        rate = rospy.Rate(10)
        while abs(front_avg_dist - TARGET_BOARD_DIST) > 0.05:
            front_scan, _ = LidarSampler.lidar_front()
            front_avg_dist = np.mean(front_scan)
            cursor = self.cursor_msg
            diff_adj = (cursor.image_width / 2) - cursor.cursor_loc 

            cmd.linear.x = direction * 0.1 * min(front_avg_dist - TARGET_BOARD_DIST, 1)
            cmd.angular.z = direction * 0.001 * diff_adj
            self.movement_pub.publish(cmd)

            rate.sleep()
 

    def cursor_position_callback(self, cursor):
        """
        Callback method for positioning the cursor.
        """

        self.cursor_locator = cursor 



if __name__ == "__main__":
    node = RobotMathControlNode()
    node.run()