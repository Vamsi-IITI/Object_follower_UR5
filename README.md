# Object_follower_UR5
FSM Internship : Robotics Project . Software package for object tracking and visual servoing of robotic arm on cobot

## Tasks
- [ ] Visualizing Robot model in Gazebo
- [ ] Motion planning of arm using Moveit
- [ ] Detection of object using camera
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
catkin_make
```
*Note : If error like this : ' **Invoking "make -j16 -l16" failed** ' comes then please use command ```catkin_make -j4``` ( if your pc has 4 cores ) or ```catkin_make -j8``` ( if your pc has 8 cores )*
```
source devel/setup.bash
```
```
roslaunch robot_arm ur5_empty_world.launch
```
## Useful links
1. https://github.com/ros-industrial/universal_robot.git
2. https://github.com/DH-Robotics/dh_gripper_ros.git
3. https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html
