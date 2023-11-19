#!/usr/bin/env python3

import rospy
import moveit_commander


class Debug:
    def __init__(self):
        rospy.init_node("moveit_debug")
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")


    def run(self):
        rate = rospy.Rate(.1)

        while not rospy.is_shutdown():
            print("ARM", self.move_group_arm.get_current_pose().pose.position)
            rate.sleep()

        rospy.spin()


if __name__ == "__main__":
    d = Debug()
    d.run()