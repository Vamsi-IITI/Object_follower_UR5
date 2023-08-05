#!/usr/bin/env python3

import rospy
import sys
import moveit_commander
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import numpy as np
import tf
# import actionlib
# import moveit_msgs.msg

def get_user_input():
    rospy.loginfo("Enter the coordinates of the target location")
    x = float(input("Enter the x-coordinate: "))
    y = float(input("Enter the y-coordinate: "))
    z = float(input("Enter the z-coordinate: "))
    return x, y, z

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

    # Get target location from user input
    x, y, z = get_user_input()

    # Set the target pose (position and orientation) where you want the UR5 arm to move
    target_pose = Pose()
    target_pose.position.x = x
    target_pose.position.y = y
    target_pose.position.z = z
    
    # Set the target gripper orientation (horizontal pose)

    if target_pose.position.x != 0.0 :                                                   
        yaw_angle =  np.arctan(target_pose.position.y / target_pose.position.x)          # Rotation angle around the vertical axis (z-axis)
    else :                                                                               # Prevent Zero Division error
        if target_pose.position.y > 0 :
            yaw_angle = np.pi/2                                                           # Rotation angle around the vertical axis (z-axis)
        else :
            yaw_angle = -np.pi/2                                                          # Rotation angle around the vertical axis (z-axis)
    
    pitch_angle = np.pi/2                                                      # Rotation angle around the lateral axis (y-axis) # This makes gripper horizontal 
    roll_angle = 0.0                                                           # Rotation angle around the longitudinal axis (x-axis)

    # Create a quaternion from the Euler angles
    quaternion = tf.transformations.quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)
    # Normalize the quaternion to ensure it is a unit quaternion
    quaternion = tf.transformations.unit_vector(quaternion)

    # Convert to geometry_msgs/Quaternion
    target_pose.orientation = Quaternion(*quaternion)

    # Set the target gripper offset to prevent collision of gripper and target ( here box )
    offset_x = 0.15*(np.cos(yaw_angle))                    # Offset in x-direction w.r.t. the target pose
    offset_y = 0.15*(np.sin(yaw_angle))                    # Offset in y-direction w.r.t. the target pose

    target_pose.position.x = target_pose.position.x - offset_x
    target_pose.position.y = target_pose.position.y - offset_y

    # Set the planning target pose
    arm.set_pose_target(target_pose)

    # Use Anytime Path Shortening as the planner
    arm.set_planner_id("AnytimePathShortening")

    # Plan the trajectory
    plan = arm.plan()
    # print(plan)
    arm.go(wait=True)
    rospy.loginfo("UR5 arm moved to the target pose!")

    rospy.sleep(20)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass