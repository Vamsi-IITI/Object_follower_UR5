#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose
from robot_arm.srv import target_pose, target_poseResponse

def handle_move_arm_to_target(req):
    # Set the target pose (position and orientation) where you want the UR5 arm to move
    target_pose = Pose()
    target_pose.position.x = req.x
    target_pose.position.y = req.y
    target_pose.position.z = req.z + 0.1
    target_pose.orientation.w = 1

    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize the move group for the ur5_arm
    arm = moveit_commander.MoveGroupCommander('arm')

    # Set the reference frame for pose targets
    reference_frame = "base_link"

    # Set the ur5_arm reference frame accordingly
    arm.set_pose_reference_frame(reference_frame)

    # Allow replanning to increase the odds of a solution
    arm.allow_replanning(True)

    # Allow some leeway in position (meters) and orientation (radians)
    arm.set_goal_position_tolerance(0.1)
    arm.set_goal_orientation_tolerance(0.1)

    # Set the planning target pose
    arm.set_pose_target(target_pose)

    # Use Anytime Path Shortening as the planner
    arm.set_planner_id("AnytimePathShortening")

    # Plan the trajectory
    plan = arm.plan()
    arm.go(wait=True)
    rospy.loginfo("UR5 arm moved to the target pose!")

    return target_poseResponse(success=True, message="Task completed successfully")

def visual_servoing_server():
    rospy.init_node('pose_based_visual_servoing_server')
    rospy.Service('move_arm_to_target', target_pose, handle_move_arm_to_target)
    rospy.spin()

if __name__ == '__main__':
    visual_servoing_server()
