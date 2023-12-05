#!/usr/bin/env python3

import os

import cv2, cv_bridge
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

import test
from robot_math_utils import LidarSampler
from robot_math_equation_solver.msg import CursorLocate


# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) 


TARGET_BOARD_DIST = 0.3
TARGET_VIEWING_DIST = 0.7


class RobotMathControlNode:
    """
    Represents central control of the math solving process. Controls movement
    to and from board, solving of math equation, and writing the answer.
    """


    def __init__(self):
        rospy.init_node("robot_math_master_control")

        # Start robot movements publisher
        self.movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10, latch=True)

        # Start drawing movements publisher
        self.drawing_pub = rospy.Publisher("/robot_math/math_strings", String, queue_size=10, latch=True)

        # Start manipulator subscriber
        self.manipulator_sub = rospy.Subscriber("/robot_math/manipulator_busy", Bool, self.manipulator_callback)
        self.manipulator_busy = False
        
        # Start cursor locator subscriber
        self.cursor_locator = rospy.Subscriber("/robot_math/cursor_position", CursorLocate, self.cursor_position_callback) 
        self.cursor_msg = CursorLocate(cursor_loc=-1,image_width=0)

        # Set up vision
        self.bridge = cv_bridge.CvBridge()


    def run(self):
        """
        Run the math solving process.
        """

        while not rospy.is_shutdown():

            #self.move_to("VIEWING_POS")

            #self.adjust_for_viewing()

            answer = self.run_inference()

            #self.move_to("DRAWING_POS")

            self.draw_answer(answer)

            rospy.sleep(10)


    def run_inference(self):
        """
        Solve the math equation using trained nn.
        """

        # Sample image from camera
        img = None
        while img is None:
            img = rospy.wait_for_message("camera/rgb/image_raw", Image, timeout=1)
        img = self.bridge.imgmsg_to_cv2(img,desired_encoding='bgr8')

        # Write image to data directory and run inference on it
        cv2.imwrite(path_prefix + "/data/equation.jpg", img)
        equation = test.equation_from_image(path_prefix + "/data/equation.jpg")

        return str(test.process_and_predict_answer_from_cropped_images(equation))


    def draw_answer(self, answer):
        """
        Draw answer using inverse kinematics.
        """
        
        self.manipulator_busy = True 

        self.drawing_pub.publish(answer)

        rate = rospy.Rate(2)
        while self.manipulator_busy == True:
            rate.sleep()


    def adjust_for_viewing(self):
        """
        Rotate robot to position for viewing equation.
        """

        cmd = Twist()
        cmd.linear.x = 0

        # Initialize orientation
        cursor = self.cursor_msg
        diff_adj = (cursor.image_width * 2/3) - cursor.cursor_loc 
        rate = rospy.Rate(10)

        # Rotate until cursor is 2/3 from the left side of the robots image feed
        while cursor.cursor_loc == -1 or diff_adj > 10 :
            cursor = self.cursor_msg

            if cursor.cursor_loc != -1:
                diff_adj = (cursor.image_width * 2/3) - cursor.cursor_loc 
                cmd.angular.z = min(0.001 * diff_adj, 0.1)
            else:
                cmd.angular.z = 0.1

            self.movement_pub.publish(cmd)
            rate.sleep()

        cmd.angular.z = 0.0
        self.movement_pub.publish(cmd)


    def move_to(self, command):
        """
        Move the robot to and from the drawing and viewing positions.
        """

        # Set target distance
        if command == "DRAWING_POS":
            target_dist = TARGET_BOARD_DIST
        elif command == "VIEWING_POS":
            target_dist = TARGET_VIEWING_DIST

        cmd = Twist()
        front_avg_dist = 999

        # Use proportional control to move to position
        rate = rospy.Rate(10)
        while abs(front_avg_dist - target_dist) > 0.05:
            # Compare distance to board and target
            front_scan, _ = LidarSampler.lidar_front()
            front_avg_dist = np.mean(front_scan)

            # Set direction of travel relative to target distance
            if front_avg_dist - target_dist > 0:
                direction = 1
            else:
                direction = -1

            # Adjust orientation to cursor
            cursor = self.cursor_msg
            diff_adj = (cursor.image_width / 2) - cursor.cursor_loc 

            # Publish movement command using proportional control
            cmd.linear.x = direction * 0.2 * min(abs(front_avg_dist - target_dist), 1)
            cmd.angular.z = direction * 0.001 * diff_adj
            self.movement_pub.publish(cmd)

            rate.sleep()
        
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.movement_pub.publish(cmd)
 

    def cursor_position_callback(self, cursor):
        """
        Callback method for positioning the cursor.
        """

        self.cursor_msg = cursor 
    

    def manipulator_callback(self, is_busy):
        """
        Callback method for manipulator movements.
        """
        self.manipulator_busy = is_busy.data


if __name__ == "__main__":
    node = RobotMathControlNode()
    node.run()