# Object_follower_UR5
FSM Internship : Robotics Project . Software package for object tracking and visual servoing of robotic arm on cobot

## Tasks
- [X] Visualizing Robot model in Gazebo
- [X] Motion planning of arm using Moveit
- [X] Detection of object using camera
- [ ] Object tracking and Visual servoing of robot arm
- [ ] Motion prediction ( to improve tracking ) . Use of filters like Kalman filters , etc.

## Instructions for use
```
git clone https://github.com/Vamsi-IITI/Object_follower_UR5.git
```
```
cd Object_follower_UR5
```
```
rosdep install --from-paths src --ignore-src -r -y
```
```
catkin_make
```
*Note : If error like this : ' **Invoking "make -j16 -l16" failed** ' comes then please use command ```catkin_make -j4``` ( if your pc has 4 cores ) or ```catkin_make -j8``` ( if your pc has 8 cores )*
```
source devel/setup.bash
```
*For **robot visualization** :*
```
roslaunch robot_arm ur5_empty_world.launch
```
*Note : PID Gains of Gripper can be tuned further using Dynamic Reconfiguration ```rosrun rqt_reconfigure rqt_reconfigure```*

*For **moveit rviz simulation** :*
```
roslaunch robot_arm_moveit_config demo.launch 
```
#### Darknet ROS Installation 
[darknet_ros](https://github.com/leggedrobotics/darknet_ros)
```
cd ~/Object_follower_UR5/src
```
```
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
```
```
cd ~/Object_follower_UR5
```
```
catkin_make -DCMAKE_BUILD_TYPE=Release
```
## Useful links
1. [Universal Robotics UR5 arm](https://github.com/ros-industrial/universal_robot.git)
2. [DH Robotics AG 95 Gripper](https://github.com/DH-Robotics/dh_gripper_ros.git)
3. [Darknet ROS](https://github.com/leggedrobotics/darknet_ros)
4. [Moveit Tutorials](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
5. https://www.youtube.com/watch?v=b4T577d39dE&t=281s
6. https://youtu.be/1DTO4tzjJ0I
7. https://answers.ros.org/question/372866/how-to-save-and-read-load-a-trajectory/
8. [Yolo Darknet Guide](https://medium.com/geekculture/yolov4-darknet-installation-and-usage-on-your-system-windows-linux-8dec2cea6e81#a59a)

## Testing ennvironment 
```
Lenovo Ideapad 5 @ Ryzen 7 5700U
CPU : 8 cores , 16 threads . Integrated Graphics .
16 GB RAM , 512 GB SSD
OS : Ubuntu 20.04.6 LTS ( Focal Fossa )
ROS Distro : Noetic
```
