#!/usr/bin/env python3

import rospy, sys
import moveit_commander
from geometry_msgs.msg import Pose

from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.msg import Grasp
# from std_msgs.msg import Header

# from trajectory_msgs.msg import JointTrajectory
# from trajectory_msgs.msg import JointTrajectoryPoint


def main():

    rospy.init_node('ur5_rrt_connect_example', anonymous=True)

    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize the move group for the ur5_arm
    arm = moveit_commander.MoveGroupCommander('arm')

    # Get the name of the end-effector link
    end_effector_link = arm.get_end_effector_link()

    # Set the reference frame for pose targets
    reference_frame = "base_link"

    # Set the ur5_arm reference frame accordingly
    arm.set_pose_reference_frame(reference_frame)

    # Allow replanning to increase the odds of a solution
    arm.allow_replanning(True)

    # Allow some leeway in position (meters) and orientation (radians)
    arm.set_goal_position_tolerance(0.1)
    arm.set_goal_orientation_tolerance(0.1)

    # Set the internal state to the current state
    arm.set_start_state_to_current_state()

    # Go to reference pose
    arm.set_named_target("reference_pose")
    plan = arm.go()
    rospy.loginfo("UR5 arm moved to the reference pose!")
    rospy.sleep(20)

    # Get the current pose so we can add it as a waypoint
    # start_pose = arm.get_current_pose(end_effector_link).pose

    # Set the internal state to the current state
    arm.set_start_state_to_current_state()

    # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

    # Set the target pose (position and orientation) where you want the UR5 arm to move
    target_pose = Pose()
    target_pose.position.x = 0.5
    target_pose.position.y = 0.0
    target_pose.position.z = 0.5
    target_pose.orientation.w = 1.0

    # Set the planning target pose
    arm.set_pose_target(target_pose)

    # Use RRT Connect as the planner
    arm.set_planner_id("RRTConnect")

    # Plan the trajectory
    plan = arm.plan()
    # print(plan)

    # Execute the trajectory
    # arm.execute(plan, wait=True)
    arm.go()
    rospy.loginfo("UR5 arm moved to the goal position!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass