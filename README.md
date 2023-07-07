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
>*Note : If error like this : ' **Invoking "make -j16 -l16" failed** ' comes then please use command **```catkin_make -j4```** ( if your pc has 4 cores ) or **```catkin_make -j8```** ( if your pc has 8 cores )*
```
source devel/setup.bash
```
*For **robot visualization** :* ( Task 1 )
```
roslaunch robot_arm ur5_empty_world.launch
```
>*Note : PID Gains of Gripper can be tuned further using Dynamic Reconfiguration **```rosrun rqt_reconfigure rqt_reconfigure```***

*For **moveit rviz simulation** :* ( Task 2 )
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
## For testing performance of trained weights :
> 
> ```
> cd ~
> git clone https://github.com/AlexeyAB/darknet.git
> cd darknet
> ```
> ( Make sure you have downloaded necessary libraries , please refer to these guides [1](https://robocademy.com/2020/05/01/a-gentle-introduction-to-yolo-v4-for-object-detection-in-ubuntu-20-04/) and [2](https://medium.com/geekculture/yolov4-darknet-installation-and-usage-on-your-system-windows-linux-8dec2cea6e81#a59a) )
>
> Make changes in Makefile and save them using gedit or code (vs code)
> ```
> code Makefile
> ```
> For CPU build , set following parameters in Makefile :
>> ```GPU=0
>> CUDNN=0
>> CUDNN_HALF=0
>> OPENCV=1
>> AVX=1
>> OPENMP=1
>> LIBSO=1  
>> ZED_CAMERA=0
>> ZED_CAMERA_v2_8=0
>> ```
>> Save the edited Makefile
> ```
> make
> ```
> Now copy obj.data , obj.names and yolov3_training.cfg files from ~/Object_follower_UR5/src/yolov3/cfg folder to cfg folder of darknet directory. Also place weights file in darknet directory
> Now place any test image ( for example here it is two_boxes.png ) in darknet directory and run following command :
> ```
> cd ~/darknet
> ./darknet detector test cfg/obj.data cfg/yolov3_training.cfg yolov3_training.weights two_boxes.png
> ```
> Watch the predictions of model! (**Task 3 - Object Detection**)
> 
> ![predictions](https://github.com/Vamsi-IITI/Object_follower_UR5/assets/92263050/5b41d583-c8f7-470b-807a-4629b3b6628e)


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
