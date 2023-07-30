#!/usr/bin/env python3

import rospy
import sys
import tf
import numpy as np
import moveit_commander
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from robot_arm.srv import target_pose, target_poseResponse

def handle_move_arm_to_target(req):

    #################################### Ideal code #####################################

    #### For the case when the target location is determined perfectly. When there are no inaccuracies in sensor measurement and tf transforms

    # # Set the target pose (position and orientation) where you want the UR5 arm to move
    # target_pose = Pose()
    # target_pose.position.x = req.x     
    # target_pose.position.y = req.y      
    # target_pose.position.z = req.z 

    # # Set the target gripper orientation (horizontal pose)
    # if target_pose.position.x != 0.0 :
    #     yaw_angle =  np.arctan(target_pose.position.y / target_pose.position.x)          # Rotation angle around the vertical axis (z-axis)
    # else :
    #     if target_pose.position.y > 0 :
    #         yaw_angle = np.pi/2                                                           # Rotation angle around the vertical axis (z-axis)
    #     else :
    #         yaw_angle = -np.pi/2 

    # pitch_angle = np.pi/2                                                            # Rotation angle around the lateral axis (y-axis)
    # roll_angle = 0.0                                                                 # Rotation angle around the longitudinal axis (x-axis)

    # # Create a quaternion from the Euler angles
    # quaternion = tf.transformations.quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)
    # # Normalize the quaternion to ensure it is a unit quaternion
    # quaternion = tf.transformations.unit_vector(quaternion)

    # # Convert to geometry_msgs/Quaternion
    # target_pose.orientation = Quaternion(*quaternion)

    # # Set the target gripper offset to prevent collision of gripper and target ( here box )
    # offset_x = 0.15*(np.cos(yaw_angle))                    # Offset in x-direction w.r.t. the target pose
    # offset_y = 0.15*(np.sin(yaw_angle))                    # Offset in y-direction w.r.t. the target pose

    # target_pose.position.x = target_pose.position.x - offset_x
    # target_pose.position.y = target_pose.position.y - offset_y

    # print(target_pose)

    #####################################################################################  

    # Set the target pose (position and orientation) where you want the UR5 arm to move
    target_pose = Pose()
    target_pose.position.x = req.x + 0.2        ## x and y correspond to coordinates of center point of bounding box , the closest face of box is closer to arm than these values
    target_pose.position.y = req.y - 0.15       ## had to change to nullify inaccuracies of position
    target_pose.position.z = 0.08               ## z values obtained from transformations have slight error as of now                        

    # Set the target gripper orientation (horizontal pose)                           
    yaw_angle =  np.pi/2       # Had to set manually due to inaccuracy in target location  # Rotation angle around the vertical axis (z-axis)
    pitch_angle = np.pi/2                                                                  # Rotation angle around the lateral axis (y-axis)
    roll_angle = 0.0                                                                       # Rotation angle around the longitudinal axis (x-axis)

    # Create a quaternion from the Euler angles
    quaternion = tf.transformations.quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)
    # Normalize the quaternion to ensure it is a unit quaternion
    quaternion = tf.transformations.unit_vector(quaternion)

    # Convert to geometry_msgs/Quaternion
    target_pose.orientation = Quaternion(*quaternion)

    print(target_pose)

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

    # Set the planning target pose
    arm.set_pose_target(target_pose)

    # Use Anytime Path Shortening as the planner
    arm.set_planner_id("AnytimePathShortening")

    # # Plan the trajectory
    # plan = arm.plan()
    arm.go(wait=True)
    rospy.loginfo("UR5 arm moved to the target pose!")

    rospy.sleep(15)
    # Shut down the node after executing the trajectory
    # rospy.signal_shutdown("Trajectory execution completed.")

    return target_poseResponse("Task completed successfully")

def visual_servoing_server():
    rospy.init_node('pose_based_visual_servoing_server')
    rospy.Service('move_arm_to_target', target_pose, handle_move_arm_to_target)
    rospy.spin()

if __name__ == '__main__':
    visual_servoing_server()
