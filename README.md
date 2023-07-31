# Object_follower_UR5
FSM Internship : Robotics Project . Software package for object tracking and visual servoing of robotic arm on cobot

## Tasks
- [X] Visualizing Robot model in Gazebo
- [X] Motion planning of arm in Rviz using Moveit
- [X] Object detection and localization
- [X] 3D Pose estimation of object and Visual servoing of UR5 robotic arm
- [ ] Motion prediction ( to improve tracking ) . Use of filters like Kalman filters , etc.

## Instructions for use
```
git clone https://github.com/Vamsi-IITI/Object_follower_UR5.git
```
```
pip install ultralytics
```
```
cd ~/Object_follower_UR5
```
```
rosdep install --from-paths src --ignore-src -r -y
```
```
catkin_make
```
>*Note : If error like this : ' **Invoking "make -j16 -l16" failed** ' comes then please use command **```catkin_make -j4```** ( if your pc has 4 cores ) or **```catkin_make -j8```** ( if your pc has 8 cores )*
```
source devel/setup.bash
```

#### Robot visualization : *( Task 1 )*
```
cd ~/Object_follower_UR5
source devel/setup.bash
roslaunch robot_arm ur5_empty_world.launch
```
>*Note : PID Gains of Gripper can be tuned further using Dynamic Reconfiguration **```rosrun rqt_reconfigure rqt_reconfigure```***

> For manual control of robot, open another terminal and run command **```rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller```**

#### Moveit Rviz simulation : *( Task 2 )*
```
cd ~/Object_follower_UR5
source devel/setup.bash
roslaunch arm_moveit_config demo.launch 
```

#### Teleoperation of Robot Arm to User-defined Target :
Run the shell script:
```s
cd ~/Object_follower_UR5
source devel/setup.bash
./src/shell_scripts/pbvs.sh
```
**OR**
```s
cd ~/Object_follower_UR5
source devel/setup.bash
roslaunch robot_arm ur5_empty_world.launch
```
Wait for 5-6 seconds. Then, open another terminal and run following commands:
```s
source devel/setup.bash
roslaunch arm_moveit_config moveit_planning_execution.launch
```
Wait for 5-6 seconds. Then, open another terminal and run following commands:
```s
source devel/setup.bash
roslaunch robot_arm pbvs.launch
```
Enter target coordinates and let the script runs. Sometimes control gets aborted, but even then trajectory is executed. If arm doesn't reach correct position. Restart and rerun all the commands.
> Note : pbvs.py script has ideal code for motion planning and execution of robotic arm towards a specified target
> While robot_arm_manipulation.py script has to take into account the inaccuracies in location of target

#### 3D Pose estimation of object and Pose-based Visual servoing of UR5 robotic arm : *( Task 4 )*
Simply just run the shell script: (works for gnome terminal)
```s
cd ~/Object_follower_UR5
source devel/setup.bash
./src/shell_scripts/object_follower_ur5.sh
```
**OR**
```s
cd ~/Object_follower_UR5
source devel/setup.bash
roslaunch robot_arm box_world.launch
```
Wait for 5-6 seconds. Then, open another terminal and run following commands:
```s
source devel/setup.bash
roslaunch arm_moveit_config moveit_planning_execution.launch
```
Wait for 5-6 seconds. Then, open another terminal and run following commands:
```s
source devel/setup.bash
roslaunch robot_arm go_to_referencepos.launch
```
Wait till the execution of above script ends (~35 secs). Make sure box is detected. Then, open another terminal and run following commands:
```s
source devel/setup.bash
roslaunch robot_arm robot_arm_manipulation.launch"
```
Wait for 5-6 seconds. Then, open another terminal and run following commands:
```s
source devel/setup.bash
roslaunch robot_arm move_arm_client.launch
```

## Useful links
1. [Universal Robotics UR5 arm](https://github.com/ros-industrial/universal_robot.git)
2. [DH Robotics AG 95 Gripper](https://github.com/DH-Robotics/dh_gripper_ros.git)
3. [Darknet ROS](https://github.com/leggedrobotics/darknet_ros)
4. [Moveit Tutorials](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
5. [MoveIt Training for Beginners , By Construct](https://www.youtube.com/watch?v=b4T577d39dE&t=281s)
6. [MoveIt! Robot Manipulators , By Robotics with Sakshay](https://youtu.be/1DTO4tzjJ0I)
7. https://answers.ros.org/question/372866/how-to-save-and-read-load-a-trajectory/
8. [Yolo Darknet Guide](https://medium.com/geekculture/yolov4-darknet-installation-and-usage-on-your-system-windows-linux-8dec2cea6e81#a59a)
9. [A Gentle Introduction to YOLO v4 for Object detection in Ubuntu 20.04](https://robocademy.com/2020/05/01/a-gentle-introduction-to-yolo-v4-for-object-detection-in-ubuntu-20-04/)
10. [YOLO Object Detection | ROS Developers Live Class , By Construct](https://www.youtube.com/live/dB0Sijo0RLs?feature=share)

## Testing environment 
```
Lenovo Ideapad 5 @ Ryzen 7 5700U
CPU : 8 cores , 16 threads . Integrated Graphics .
16 GB RAM , 512 GB SSD
OS : Ubuntu 20.04.6 LTS ( Focal Fossa )
ROS Distro : Noetic
```
