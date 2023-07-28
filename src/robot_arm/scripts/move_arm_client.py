#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, Point
from yolov8_ros_msgs.msg import  ObjectLocations
from robot_arm.srv import target_pose

# Reference taken from --> tf2_ros_example.py: example showing how to use tf2_ros API
# Link : https://gist.github.com/ravijo/cb560eeb1b514a13dc899aef5e6300c0

class ObjectLocationConverterNode:

    def __init__(self):

        rospy.init_node('target_localisation_node')

        # Subscribe to the ObjectLocation topic
        rospy.Subscriber('/yolov8/ObjectLocation', ObjectLocations, self.object_location_callback)

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
    
    def move_robot_arm_client(self, x, y, z):

        # Send the target location to ros service "move_arm_to_target"
        rospy.wait_for_service('/move_arm_to_target')

        try:
            move_arm_to_target = rospy.ServiceProxy('/move_arm_to_target', target_pose)
            success , message = move_arm_to_target(x, y, z)
            return success, message
            # if success:
            #     print(message)
            # else:
            #     rospy.loginfo("Task failed")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

    def object_location_callback(self, object_location_msg):
        
        # Check if any objects are detected
        if len(object_location_msg.object_location) == 0:
            rospy.loginfo("No objects detected.")
            return

        # Find the object with the highest probability
        highest_prob_object = max(object_location_msg.object_location, key=lambda x: x.probability)

        # Get the object location in the camera frame
        object_location_cam = Point(highest_prob_object.x, highest_prob_object.y, highest_prob_object.z)

        # Convert the object location from the camera frame to the world frame / base_link frame
        transformation = self.get_transformation(self.source_frame, self.target_frame)
        point_wrt_target = self.transform_point(transformation, object_location_cam)

        # Send coordinates to move_robot_arm_client
        success, message = self.move_robot_arm_client(point_wrt_target[0], point_wrt_target[1], point_wrt_target[2])
        
def main():

    converter_node = ObjectLocationConverterNode()
    rospy.spin()

if __name__ == '__main__':
    main()