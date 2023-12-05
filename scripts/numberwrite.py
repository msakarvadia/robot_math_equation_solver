#!/usr/bin/env python3

import math
import subprocess

import numpy as np

import rospy
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectoryPoint

from robot_math_equation_solver.srv import CharacterPath


def inv_kin(x, y, z):
    """
    """

    # Set lengths
    l1 = 0.077 + 0.141 - 0.03        # arm-base + turtlebot height - lidar height
    l2 = 0.130                       # upper arm length 
    l3 = 0.124 + 0.126 + 0.185       # forearm length + gripper + pen

    # Calculate intermediary values
    r = math.sqrt(x**2 + (z - l1)**2)
    phi1 = math.acos((l2**2 + r**2 - l3**2)/(2 * l2 * r))
    phi2 = math.acos((l2**2 + l3**2 - r**2)/(2 * l2 * l3))
    phi3 = math.atan2((z - l1), x)

    # Calculate theta1, theta2, theta3
    theta1 = math.atan2(y, x)
    theta2 = phi3 + phi1
    theta3 = phi2 - math.pi 

    # Map model's angles to our arm's configuration
    theta2 = -1 * (theta2 - math.pi / 2)
    theta3 = -1 * (theta3 + math.pi / 2)

    # Adjust angles for additional l2 length
    offset = math.asin(0.024/l2)
    theta2 -= offset
    theta3 += offset

    return theta1, theta2, theta3


class InverseKinematicsPlanner:
    """
    """

    def __init__(self):
        # initialize this node
        rospy.init_node("robot_math_manipulator_movement")

        # Disable allowed start tolerance for moveit
        manipulator_setup = """
        rosservice call /move_group/trajectory_execution/set_parameters "config:
        doubles:
            - {name: 'allowed_start_tolerance', value: 0.0}"
        """
        subprocess.call(manipulator_setup, shell=True)

        # the interface to the group of joints making up the turtlebot3
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Wait for character path service
        rospy.wait_for_service("/robot_math/character_path_service")
        self.path_client = rospy.ServiceProxy("/robot_math/character_path_service", CharacterPath)

        # Start publisher for signalling finish of manipulator movements
        self.is_busy_pub = rospy.Publisher("/robot_math/manipulator_busy", Bool, queue_size=10, latch=True)
        self.is_busy = False


    def write_num_trajectory(self, points):
        """
        """

        # Create joint moveit trajectory
        trajectory = RobotTrajectory()
        trajectory.joint_trajectory.joint_names = self.move_group_arm.get_active_joints()

        # Move to start point
        start = points[0]
        t1, t2, t3 = inv_kin(start[0], start[1], start[2])
        self.move_group_arm.go([t1, t2, t3, 0.0], wait=True)
        rospy.sleep(5)

        # Create joint trajectory using inverse kinematics
        time_increment = 0.01
        for i, point in enumerate(points):
            t1, t2, t3 = inv_kin(point[0], point[1], point[2])
            point = JointTrajectoryPoint()
            point.positions = [t1, t2, t3, 0.0]
            point.time_from_start = rospy.Duration(i * time_increment)
            trajectory.joint_trajectory.points.append(point)

        # Execute trajectory
        self.move_group_arm.execute(trajectory, wait=True)
        rospy.sleep(time_increment * len(points) + 7)


    def run(self):
        """
        """

        rate = rospy.Rate(0.33)

        while not rospy.is_shutdown():
            # Request a character point path from server
            try:
                path_resp = self.path_client(request=True)
                flattened_3D_array = path_resp.point_path
                points = np.array(flattened_3D_array).reshape(len(flattened_3D_array) // 3, 3)

            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

            # If server not empty, write the character and report manipulator busy
            if points.size > 0:
                if self.is_busy == False:
                    self.is_busy_pub.publish(True)
                    self.is_busy = True
                self.write_num_trajectory(points)

            elif self.is_busy == True:
                self.is_busy_pub.publish(False)
                self.is_busy = False

            rate.sleep()


if __name__ == "__main__":
    node = InverseKinematicsPlanner()
    node.run()