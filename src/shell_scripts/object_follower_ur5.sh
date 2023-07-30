#!/bin/sh

# launch box_world.launch to deploy ur5 robot arm with gripper and box in environment
gnome-terminal -- bash -c "source ~/Object_follower_UR5/devel/setup.bash";
gnome-terminal -- bash -c "roslaunch robot_arm box_world.launch" & 

sleep 7

# launch moveit_planning_execution.launch to start moveit
gnome-terminal -- bash -c "source ~/Object_follower_UR5/devel/setup.bash";
gnome-terminal -- bash -c "roslaunch arm_moveit_config moveit_planning_execution.launch" &

sleep 7

# launch go_to_referencepos.launch to move robot arm to reference position  
gnome-terminal -- bash -c "source ~/Object_follower_UR5/devel/setup.bash";
gnome-terminal -- bash -c "roslaunch robot_arm go_to_referencepos.launch" &

sleep 40

# launch robot_arm_manipulation.launch to start move_arm_to_target service 
gnome-terminal -- bash -c "source ~/Object_follower_UR5/devel/setup.bash";
gnome-terminal -- bash -c "roslaunch robot_arm robot_arm_manipulation.launch" &

sleep 6

# launch move_arm_client.launch to start target_localisation node
gnome-terminal -- bash -c "source ~/Object_follower_UR5/devel/setup.bash";
gnome-terminal -- bash -c "roslaunch robot_arm move_arm_client.launch"