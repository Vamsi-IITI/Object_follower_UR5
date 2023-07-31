#!/bin/sh

# launch box_world.launch to deploy ur5 robot arm with gripper and box in environment
gnome-terminal -- bash -c "source ~/Object_follower_UR5/devel/setup.bash";
gnome-terminal -- bash -c "roslaunch robot_arm ur5_empty_world.launch" & 

sleep 10

# launch moveit_planning_execution.launch to start moveit
gnome-terminal -- bash -c "source ~/Object_follower_UR5/devel/setup.bash";
gnome-terminal -- bash -c "roslaunch arm_moveit_config moveit_planning_execution.launch" &

sleep 7

# launch go_to_referencepos.launch to move robot arm to reference position  
gnome-terminal -- bash -c "source ~/Object_follower_UR5/devel/setup.bash";
gnome-terminal -- bash -c "roslaunch robot_arm pbvs.launch"