#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('inverse_kinematics', anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 0.184445195028
    pose_goal.position.x = 0.437686078966
    pose_goal.position.y = 0.138975159367
    pose_goal.position.z = 0.480383719667

    
    move_group.set_pose_target(pose_goal)

    plan = move_group.go(wait=True)
    move_group.stop()

    move_group.clear_pose_targets()


    





