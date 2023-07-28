#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose
# import actionlib
# import moveit_msgs.msg

def main():
    rospy.init_node('ur5_arm_manipulation', anonymous=True)

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

    # Go to reference pose
    arm.set_named_target("reference_pose")
    arm.go(wait=True)  # Use 'wait=True' to wait for the execution to complete
    rospy.loginfo("UR5 arm moved to the reference pose!")

    # Set the target pose (position and orientation) where you want the UR5 arm to move
    target_pose = Pose()
    target_pose.position.x = 0.5
    target_pose.position.y = 0.0
    target_pose.position.z = 0.5
    target_pose.orientation.w = 1.0

    # Set the planning target pose
    arm.set_pose_target(target_pose)

    # Use Anytime Path Shortening as the planner
    arm.set_planner_id("AnytimePathShortening")

    # Plan the trajectory
    plan = arm.plan()
    # print(plan)
    arm.go(wait=True)
    rospy.loginfo("UR5 arm moved to the target pose!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass