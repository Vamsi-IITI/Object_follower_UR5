#!/usr/bin/env python3

import rospy
import sys
import moveit_commander

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

    # # Scaling factor for optionally reducing the maximum joint velocity and joint acceleration
    # # Currently both are set to 0.1 in joint_limits.yaml file of arm_moveit_config package. For max speed , set both to 1
    # arm.set_max_velocity_scaling_factor(0.5)
    # arm.set_max_acceleration_scaling_factor(0.5)

    # Use Anytime Path Shortening as the planner
    arm.set_planner_id("AnytimePathShortening")

    # Go to reference pose
    arm.set_named_target("reference_pose_y")
    plan = arm.go(wait=True)  # Use 'wait=True' to wait for the execution to complete
    rospy.loginfo("UR5 arm moved to the reference pose!")

    rospy.sleep(20)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass