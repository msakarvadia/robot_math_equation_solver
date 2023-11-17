#!/usr/bin/env python3

import rospy

import moveit_commander
import math
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Image

import sympy as sp
from sympy.solvers import solve

def inv_kin(x, y, z):

        l1 = 0.035 + 0.141              # arm-base + turtlebot height
        l2 = 0.130                      # arm length 
        l3 = 0.124 + 0.126 - 0.0545     # arm length + gripper - delta

        c3 = (x**2 + y**2 + z**2 - (l1**2 + l2**2 + l3**2) - 2 * l1 * (z - l1)) / (2 * l2 * l3)
        s3 = -np.sqrt(1 - c3)

        theta1 = math.atan2(y, x)
        theta2 = (math.atan2((z - l1) * (math.cos(theta1) - math.sin(theta1)), (x - y)) 
                  - math.atan2((s3 * l3), (c3 * l3 + l2)))
        theta3 = math.atan2(s3, c3)

        # Map calculated angles to openmanipulator joints
        theta2 = -1 * (theta2 - math.pi / 2)
        theta3 = -1 * (theta3 + math.pi / 2)

        # Adjust for openmanipulator joint 3 additional length
        theta2 -= 0.155348
        theta3 += 0.155348

        return theta1, theta2, theta3


def inv_kin_4d (x, y, z):
    d1 = 0.077
    a2 = 0.130
    a3 = 0.124
    a5 = 0.126

    theta1 = math.atan2(y,x)

    r3 = math.sqrt(x**2 + y**2)

    z3 = z - d1

    eq1 = sp.Eq(theta2 + theta3 + theta4 - total)
    eq2 = sp.Eq(r3 - (a4 * math.cos(total)) - r2)
    eq3 = sp.Eq(z3 - (a4 * math.sin(total)) - z2)

    #can be plus or minus
    eq4 = sp.Eq(math.acos((r2**2 + z2**2 - (a2**2 + a3**2))/(2*a2*a3)) - theta3)

    # r2 = (a2 * math.cos(theta2)) + (a3 * math.cos(theta2 + theta3))
    # z2 = (a2 * math.sin(theta2)) + (a3 * math.sin(theta2 + theta3))

    # r2 = (math.cos(theta2) * (a2 + (a3 * math.sin(theta3)))) - (math.sin(theta2) * (a3 * math.sin(theta3)))
    # z2 = (math.cos(theta2) * a3 * math.sin(theta3)) + (math.sin(theta2) * (a2 + (a3 * math.cos(theta3))))

    # sin2/cos2
    eq5 = sp.Eq(math.atan2(((a2 + (a3 * math.cos(theta3))) * z2 + (a3 *math.sin(theta3)) * r2) / (r2**2 + z2**2), ((a2 + (a3 * math.cos(theta3))) * r2 + (a3 *math.sin(theta3)) * z2) / (r2**2 + z2**2)) - theta2)
    eq6 = sp.Eq(total - (theta2 + theta3) - theta4)

    

    output = solve([eq1, eq2, eq3, eq4, eq5, eq6], dict=True)
    print(output)
    values = list(output[0].values())

    theta2 = values[0]
    theta3 = values[1]
    theta4 = values[2]

    return theta1, theta2, theta3, theta4





class Robot(object):

    def __init__(self):
        # initialize this node
        rospy.init_node('move_robot')

        # initalize the debugging window
        # cv2.namedWindow("window", 1)

        lin = Vector3()
        ang = Vector3()
        self.vel = Twist(linear=lin, angular=ang)

        # Define twist publisher to help the robot move
        self.cmdpublisher = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

        # subscribe to the robot's RGB camera data stream
        # self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

        # self.bridge = cv_bridge.CvBridge()

        # subscribe to the lidar scan from the robot
        # self.scan = LaserScan()
        # rospy.Subscriber("/scan", LaserScan, self.get_scan)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        # self.move_group_arm.go([0,0,0,0], wait=True)
        rospy.sleep(2)
        
        #self.move_group_arm.go([0,0,0,0], wait=True)
        #rospy.sleep(2)

        # Close the gripper
        gripper_joint_goal = [-0.009, 0.009]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()
        rospy.sleep(1)

        print("intial x,y,z")
        print("ARM", self.move_group_arm.get_current_pose().pose.position)


    def writenum(self):
        
        #test
        t1, t2, t3 = inv_kin(0.194, 0.000, 0.304)

        print(t1)
        print(t2)
        print(t3)

        arm_joint_goal = [t1, t2, t3, 0.0]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        rospy.sleep(5)

        t1, t2, t3 = inv_kin(0.194, -0.020, 0.304)

        arm_joint_goal = [t1, t2, t3, 0]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        rospy.sleep(5)

        t1, t2, t3 = inv_kin(0.194, -0.020, 0.264)

        arm_joint_goal = [t1, t2, t3, 0]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        rospy.sleep(5)

        t1, t2, t3 = inv_kin(0.194, 0.0, 0.264)

        arm_joint_goal = [t1, t2, t3, 0]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        rospy.sleep(5)

        t1, t2, t3 = inv_kin(0.194, 0.0, 0.304)

        arm_joint_goal = [t1, t2, t3, 0]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        rospy.sleep(5)




    def run(self):
        self.writenum()
        # rospy.spin()


if __name__ == "__main__":
    robot = Robot()
    robot.run()
