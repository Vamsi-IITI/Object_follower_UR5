# Object_follower_UR5
FSM Internship : Robotics Project . Software package for object tracking and visual servoing of robotic arm on cobot

## Tasks
- [X] Visualizing Robot model in Gazebo
- [X] Motion planning of arm in Rviz using Moveit
- [X] Object detection and localization
- [X] 3D Pose estimation of object and Visual servoing of UR5 robotic arm
- [ ] Object tracking using Kalman filters, etc. and Visual Servoing of arm

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

## Robot visualization : *( Task 1 )*
```s
cd ~/Object_follower_UR5
source devel/setup.bash
roslaunch robot_arm ur5_empty_world.launch
```
>*Note : PID Gains of Gripper can be tuned further using Dynamic Reconfiguration **```rosrun rqt_reconfigure rqt_reconfigure```***

> For manual control of robot, open another terminal and run command **```rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller```**

## Moveit Rviz simulation : *( Task 2 )*
```s
cd ~/Object_follower_UR5
source devel/setup.bash
roslaunch arm_moveit_config demo.launch 
```

## Robot Arm Manipulation to User-defined Target :
Run the shell script:
```s
cd ~/Object_follower_UR5
source devel/setup.bash
./src/shell_scripts/pbvs.sh
```
##### OR
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
Enter target coordinates and let the script runs. Sometimes control gets aborted, but even then trajectory is executed. If arm doesn't reach correct position. Restart and rerun all the commands. The issue is here : [Link](https://answers.ros.org/question/349087/invalid-trajectory-start-point-deviates-from-current-robot-state-more-than-001-joint/) . I couldn't find any reliable solution for this issue. Increased tolerance to 0.1 in trajectory_execution_launch.xml file in arm_moveit_config but still it occurs. Re running all scripts helps !

https://github.com/Vamsi-IITI/Object_follower_UR5/assets/92263050/40a71098-75a3-4ca2-b300-feecf34b73ae

[Video Demo](https://drive.google.com/file/d/10qvR62eNwggTM3F3-nZ5gw2PUTH4LDAz/view?usp=sharing)

##### pbvs.py script :
```
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
```

> Note : pbvs.py script has ideal code for motion planning and execution of robotic arm towards a specified target
> 
> While robot_arm_manipulation.py script has to take into account the inaccuracies in location of target

## 3D Pose estimation of object and Pose-based Visual servoing of UR5 robotic arm : *( Task 4 )*
Simply just run the shell script: (works for gnome terminal)
```s
cd ~/Object_follower_UR5
source devel/setup.bash
./src/shell_scripts/object_follower_ur5.sh
```
##### OR
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

https://github.com/Vamsi-IITI/Object_follower_UR5/assets/92263050/248d24c2-4ae3-47e2-b161-423463974d1c

[Video Demo](https://drive.google.com/file/d/10qvR62eNwggTM3F3-nZ5gw2PUTH4LDAz/view?usp=sharing)

## Object Detection and Localization : *( Task 3 )*

### Yolo v8
[yolov8_ros package](https://github.com/Vamsi-IITI/yolov8_ros) and object_location_convertor_node.py / move_arm_client.py help do object detection and localization in gazebo simulation.
Ultralytics library is used for running Yolov8 Small model ( ~ 22 MB size , FPS: 3-5 ) to detect boxes. Inverse projection transformation and tf transformation ( between camera link and base link) help in getting location of object in world. I forked yolov8_ros package , added code to enable using custom weights , added code for inverse projection transformation and added a new rostopic for publishing locations of detected objects wrt camera frame. Used tf2 ros to convert the coordinates wrt base_link frame.

Yolov8 ROS package - [Here](https://github.com/Vamsi-IITI/yolov8_ros)

Yolov8 Training notebook , weights and dataset - [Here](https://github.com/Vamsi-IITI/box_yolov8.git)

Training results : ( Trained over 25 epochs , best weights were choosen for use in project )
![image](https://github.com/Vamsi-IITI/Object_follower_UR5/assets/92263050/4cce13ef-8852-4ea1-acff-56618835fd8e)

* **Few results** :

![image](https://github.com/Vamsi-IITI/Object_follower_UR5/assets/92263050/89e87e4e-dd0b-4840-b39a-97273bbeae19)

![image](https://github.com/Vamsi-IITI/Object_follower_UR5/assets/92263050/aa646e44-b70b-4b34-acda-4df615ae8874)

https://github.com/Vamsi-IITI/Object_follower_UR5/assets/92263050/487f34f9-e45a-469e-95db-f24d260d2b75

##### yolo_ros.py script -
```
#!/usr/bin/env python3

import cv2
import torch
import numpy

import rospy
import sys

from cv_bridge import CvBridge
from ultralytics import YOLO

from sensor_msgs.msg import Image
from yolov8_ros_msgs.msg import BoundingBox, BoundingBoxes, DepthPoint, DepthPoints , Object, ObjectLocations
from std_msgs.msg import Header

from sensor_msgs.msg import CameraInfo
# from geometry_msgs.msg import Point

class yolo_class:

    def __init__(self, weights_path, classes_path, img_topic, depth_topic, queue_size, visualize):

        self._class_to_color = {}
        self.cv_bridge = CvBridge()
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        print("Using Device: ", self.device)

        self.yolo = YOLO(weights_path)
        # self.yolo = YOLO("yolov8n.pt")
        self.yolo.fuse()

        self.img_subscriber = rospy.Subscriber(img_topic, Image, self.process_img_msg)
        self.depht_subscriber = rospy.Subscriber(depth_topic, Image, self.depth_callback)

        self.camera_info_subscriber = rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.camera_info_callback)

        self.position_pub = rospy.Publisher('/yolov8/BoundingBoxes',  BoundingBoxes, queue_size=1)
        self.depth_points_pub = rospy.Publisher('/yolov8/DepthPoints',  DepthPoints, queue_size=1) 

        # Publish locations of detected objects (with respect to camera frame)
        self.object_location_pub = rospy.Publisher('/yolov8/ObjectLocation', ObjectLocations, queue_size=1)

        self.visualize = visualize
        if self.visualize:
            self.visualization_publisher = rospy.Publisher("/yolo_visualization", Image, queue_size=queue_size)

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def depth_callback(self, msg):
            self.depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def process_img_msg(self, image):

        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header

        self.depth_Points = DepthPoints()
        self.depth_Points.header = image.header

        self.object_locations = ObjectLocations()
        self.object_locations.header = image.header

        self.getImageStatus = True
        self.color_image = numpy.frombuffer(image.data, dtype=numpy.uint8).reshape(480, 640, -1)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        results = self.yolo(self.color_image, show=False, conf=0.3, verbose=False)
        self.dectshow(results, 480, 640)
        cv2.waitKey(3)

    def dectshow(self, results, height, width):

        # Intrinsic camera matrix for the raw (distorted) images.
        #     [fx  0 cx]
        # K = [ 0 fy cy]
        #     [ 0  0  1]

        # Extract camera intrinsic parameters
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        for result in results[0].boxes:
            boundingBox = BoundingBox()
            boundingBox.xmin = numpy.int64(result.xyxy[0][0].item())
            boundingBox.ymin = numpy.int64(result.xyxy[0][1].item())
            boundingBox.xmax = numpy.int64(result.xyxy[0][2].item())
            boundingBox.ymax = numpy.int64(result.xyxy[0][3].item())
            boundingBox.Class = results[0].names[result.cls.item()]
            boundingBox.probability = result.conf.item()

            # Calculate the center point of the bounding box
            depth_point = DepthPoint()
            depth_point.absolute_center_x = int(numpy.average([boundingBox.xmin, boundingBox.xmax]))
            depth_point.absolute_center_y = int(numpy.average([boundingBox.ymin, boundingBox.ymax]))
            depth_point.offset_center_x = depth_point.absolute_center_x - int(width/2)
            depth_point.offset_center_y = depth_point.absolute_center_y - int(height/2)
            depth_point.depth = self.depth_image[depth_point.absolute_center_y, depth_point.absolute_center_x]
            depth_point.Class = results[0].names[result.cls.item()]

            # Append
            self.depth_Points.depth_point.append(depth_point)
            self.boundingBoxes.bounding_boxes.append(boundingBox)

            # Calculate object coordinates using knowledge of perspective projection transformation
            object_x = (depth_point.absolute_center_x - cx) * depth_point.depth / fx
            object_y = (depth_point.absolute_center_y - cy) * depth_point.depth / fy
            object_z = depth_point.depth

            # Publish object location (with respect to camera frame)
            object_location = Object()
            object_location.Class = results[0].names[result.cls.item()]
            object_location.x = object_x
            object_location.y = object_y
            object_location.z = object_z
            object_location.probability = result.conf.item()

            ## Append
            self.object_locations.object_location.append(object_location)

        self.position_pub.publish(self.boundingBoxes)
        self.depth_points_pub.publish(self.depth_Points)

        self.object_location_pub.publish(self.object_locations)

        if self.visualize:
            self.frame = results[0].plot()
            fps = 1000.0/ results[0].speed['inference']
            cv2.putText(self.frame, f'FPS: {int(fps)}', (20,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
            self.publish_image(self.frame, height, width)
            cv2.imshow('YOLOv8', self.frame)

    def publish_image(self, imgdata, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = ''
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = numpy.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.visualization_publisher.publish(image_temp)


def main(args):

    rospy.init_node('yolov8_node', anonymous=True)

    weights_path = rospy.get_param("~weights_path", "")
    classes_path = rospy.get_param("~classes_path", "")
    img_topic =    rospy.get_param("~img_topic", "/usb_cam/image_raw")
    depth_topic =  rospy.get_param("~center_depth_topic", "/camera/depth/image_raw" )
    queue_size =   rospy.get_param("~queue_size", 1)
    visualize =    rospy.get_param("~visualize", False)
    yolo_class(weights_path,classes_path,img_topic,depth_topic,queue_size,visualize)

    rospy.loginfo("YOLOv8 initialization complete")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
```

* **yolov8_ros with pre-trained yolov8 nano weights** :
  
![image](https://github.com/Vamsi-IITI/Object_follower_UR5/assets/92263050/5ef410ac-bb1a-46dd-b9ff-5174c914b020)

* **yolov8_ros with custom weights** :

![image](https://github.com/Vamsi-IITI/Object_follower_UR5/assets/92263050/9a714994-9ef9-4057-8de2-f4c6f45d9bdd)

* **Object localization using yolov8_ros and object_location_convertor_node.py** - 

![Screenshot from 2023-07-30 15-27-45](https://github.com/Vamsi-IITI/Object_follower_UR5/assets/92263050/ccb4892e-f877-49a4-a606-017f3af99b59)

> Actual pose of center of closest face of box ( cube of side 0.1 m) : ( -0.014 , 0.744757 , 0.05 ) { look at pose of center of box shown in gazebo and calculate }
> 
> Determined pose : ( -0.18 , 0.725316 , -0.1337 )
> 
> The relative error is very less in determining y coordinate but it is large for x and z in this case

![Screenshot from 2023-07-30 15-17-08](https://github.com/Vamsi-IITI/Object_follower_UR5/assets/92263050/3e022c77-046c-4ae5-8d13-a77d8ef9d9b2)

> Actual pose of center of closest face of box ( cube of side 0.1 m) : ( 1.556259 , -0.0105 , 0.05 )
> 
> Determined pose : ( 1.554837 , 0.133255 , -0.1388 )
> 
> The relative error is very less in determining x coordinate but it is large for y and z in this case

> **Note** : I couldn't determine the cause for these inaccuracies . One reason maybe because the center of bounding box doesn't lie exactly on the center of closest face. Or there is some issue with tf transformation between camera_link and base_link. These inaccuracies are the reason why some offset were manually added in robot_arm_manipulation.py so that it can nullify effects of these errors. While pbvs.py is for ideal case when there is no error in target location.

##### object_location_convertor_node.py script -
```
#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Point
from yolov8_ros_msgs.msg import  ObjectLocations

# Reference taken from --> tf2_ros_example.py: example showing how to use tf2_ros API
# Link : https://gist.github.com/ravijo/cb560eeb1b514a13dc899aef5e6300c0

class ObjectLocationConverterNode:
    def __init__(self):
        rospy.init_node('object_location_converter_node')

        # Subscribe to the ObjectLocation topic
        rospy.Subscriber('/yolov8/ObjectLocation', ObjectLocations, self.object_location_callback)

        # Publish the object location in the world frame
        # self.world_frame_pub = rospy.Publisher('/Target_Position', PointStamped, queue_size=1)

        # define source and target frame
        self.source_frame = "camera_link"
        self.target_frame = "base_link"

    def transform_point(self, transformation, point_wrt_source):

        point_wrt_target = tf2_geometry_msgs.do_transform_point(PointStamped(point=point_wrt_source),transformation).point

        return [point_wrt_target.x, point_wrt_target.y, point_wrt_target.z]

    def get_transformation(self, source_frame, target_frame,tf_cache_duration=2.0):

        tf_buffer = tf2_ros.Buffer(rospy.Duration(tf_cache_duration))
        tf2_ros.TransformListener(tf_buffer)

        # get the tf at first available time
        try:
            # Get the transformation from the camera frame to the desired world frame (or robot arm base frame)
            # Setting rospy.Time(0) gives latest transform. Timeout = 2 secs
            transformation = tf_buffer.lookup_transform(target_frame,source_frame, rospy.Time(0), rospy.Duration(2))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr('Unable to find the transformation from %s to %s' % (source_frame, target_frame) )

        return transformation

    def object_location_callback(self, object_location_msg):
        
        # Check if any objects are detected
        if len(object_location_msg.object_location) == 0:
            rospy.loginfo("No objects detected.")
            return

        # Find the object with the highest probability
        highest_prob_object = max(object_location_msg.object_location, key=lambda x: x.probability)

        # Get the object location in the camera frame
        object_location_cam = Point(highest_prob_object.x, highest_prob_object.y, highest_prob_object.z)
        print('Object location wrt camera frame: %s' % object_location_cam)

        transformation = self.get_transformation(self.source_frame, self.target_frame)
        point_wrt_target = self.transform_point(transformation, object_location_cam)
        print('Object location wrt world frame: %s' % point_wrt_target)
        
def main():
    converter_node = ObjectLocationConverterNode()
    rospy.spin()

if __name__ == '__main__':
    main()

```

### Yolov3 Darknet 

Yolov3 trained notebook, weights and dataset - [Here](https://github.com/Vamsi-IITI/Box_detection_Yolov3_Darknet)

Though Yolov3 was successful in detecting objects in images but it didn't work properly with darknet_ros package. Also it had huge size ( ~ 250 MB ) leading to 0.1 FPS ( which is very low ) , Yolov3 Tiny had very less accuracy. darknet_ros was also causing some library conflicts on my PC. Hence I had to stop using these weights and package , and had to shift to Yolov8.

## Useful links
1. [ROS Wiki](http://wiki.ros.org/ROS/Tutorials)
2. [Universal Robotics UR5 arm](https://github.com/ros-industrial/universal_robot.git) ( For UR5 robot arm related files )
3. [DH Robotics AG 95 Gripper](https://github.com/DH-Robotics/dh_gripper_ros.git) ( For gripper files )
4. [Ultralytics](https://github.com/ultralytics/ultralytics.git) ( For Yolov8 )
5. [Yolov8 ros package](https://github.com/Vamsi-IITI/yolov8_ros) ( Added features in original repo which are now merged )
6. [Inverse Projection Transformation](https://towardsdatascience.com/inverse-projection-transformation-c866ccedef1c) ( Great article! )
7. [Moveit Tutorials](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
8. [MoveIt Training for Beginners , By Construct](https://www.youtube.com/watch?v=b4T577d39dE&t=281s) 
9. [MoveIt! Robot Manipulators , By Robotics with Sakshay](https://youtu.be/1DTO4tzjJ0I)
10. [ROS Industrial Training](https://industrial-training-master.readthedocs.io/en/melodic/_source/demo1/index.html)
11. [Move Group Commander Class Reference](http://docs.ros.org/en/jade/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html)
12. [tf2 Tutorials](http://wiki.ros.org/tf2/Tutorials)
13. [tf2 ros example](https://gist.github.com/ravijo/cb560eeb1b514a13dc899aef5e6300c0)
14. [Roboflow Yolov8 Colab Notebook](https://colab.research.google.com/github/roboflow-ai/notebooks/blob/main/notebooks/train-yolov8-object-detection-on-custom-dataset.ipynb)
15. [Yolo v8 Box Detection](https://github.com/Vamsi-IITI/box_yolov8.git) ( For Yolo v8s model , dataset and results along with some python scripts )
16. [UR5-ROS-Gazebo](https://github.com/lihuang3/ur5_ROS-Gazebo.git) ( For cartesian path planning code )
17. [Custom Manipulator Simulation in Gazebo and Motion Planning with MoveIt!](https://medium.com/@tahsincankose/custom-manipulator-simulation-in-gazebo-and-motion-planning-with-moveit-c017eef1ea90) ( A really nice article )
18. [Fetch Robotics Tutorials](https://docs.fetchrobotics.com/manipulation.html)
19. [Foxglove ROS1 Tutorials](https://foxglove.dev/blog/simulating-robotic-scenarios-with-ros1-and-gazebo)
20. [Yolo Darknet Guide](https://medium.com/geekculture/yolov4-darknet-installation-and-usage-on-your-system-windows-linux-8dec2cea6e81#a59a)
21. [A Gentle Introduction to YOLO v4 for Object detection in Ubuntu 20.04](https://robocademy.com/2020/05/01/a-gentle-introduction-to-yolo-v4-for-object-detection-in-ubuntu-20-04/)
22. [Darknet ROS](https://github.com/leggedrobotics/darknet_ros)
23. [YOLO Object Detection | ROS Developers Live Class , By Construct](https://www.youtube.com/live/dB0Sijo0RLs?feature=share)
24. [Box_detection_Yolov3_Darknet](https://github.com/Vamsi-IITI/Box_detection_Yolov3_Darknet)
25. [Coordinates and Transformations](http://motion.cs.illinois.edu/RoboticSystems/CoordinateTransformations.html)

## Testing environment 
```
Lenovo Ideapad 5 @ Ryzen 7 5700U
CPU : 8 cores , 16 threads . Integrated Graphics .
16 GB RAM , 512 GB SSD
OS : Ubuntu 20.04.6 LTS ( Focal Fossa )
ROS Distro : Noetic
```
