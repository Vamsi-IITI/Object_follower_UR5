#!/usr/bin/env python3

import numpy as np
import rospy
import tf2_ros
import tf.transformations as tf_trans
from geometry_msgs.msg import PointStamped, Point
from yolov8_ros_msgs.msg import DepthPoint, DepthPoints , Object, ObjectLocations

class ObjectLocationConverterNode:
    def __init__(self):
        rospy.init_node('object_location_converter_node')

        # Subscribe to the ObjectLocation topic
        rospy.Subscriber('/yolov8/ObjectLocation', ObjectLocations, self.object_location_callback)

        # Publish the object location in the world frame
        self.world_frame_pub = rospy.Publisher('/Target_Position', PointStamped, queue_size=1)

        # Transform listener to obtain the transformation between frames
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def object_location_callback(self, object_location_msg):
        try:
            # Check if any objects are detected
            if len(object_location_msg.object_location) == 0:
                rospy.loginfo("No objects detected.")
                return

            # Find the object with the highest probability
            highest_prob_object = max(object_location_msg.object_location, key=lambda x: x.probability)

            # Get the object location in the camera frame
            object_location_cam = [highest_prob_object.x, highest_prob_object.y, highest_prob_object.z, 1.0]

            # Get the transformation from the camera frame to the desired world frame (or robot arm base frame)
            # Setting rospy.Time(0) gives latest transform. Timeout = 3 secs
            trans = self.tf_buffer.lookup_transform("base_link", "camera_link_optical", rospy.Time(0), rospy.Duration(3.0))

            # Create a transformation matrix from the transform message
            # translation = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
            # rotation = [trans.transform.rotation.x, trans.transform.rotation.y,
            #             trans.transform.rotation.z, trans.transform.rotation.w]
            # transformation_matrix = tf_trans.concatenate_matrices(tf_trans.translation_matrix(translation),
            #                                                       tf_trans.quaternion_matrix(rotation))
            # print(translation)
            # print(rotation)
            # print(transformation_matrix)

            translation = np.array([trans.transform.translation.x,
                                    trans.transform.translation.y,
                                    trans.transform.translation.z])
            
            rotation = np.array([trans.transform.rotation.x,
                                 trans.transform.rotation.y,
                                 trans.transform.rotation.z,
                                 trans.transform.rotation.w])
            
            def create_homogeneous_matrix(translation, rotation):
                # Create a 4x4 homogeneous transformation matrix from translation and rotation
                # The rotation quaternion needs to be normalized
                rotation /= np.linalg.norm(rotation)

                # Form the rotation matrix from the quaternion
                R = tf_trans.quaternion_matrix(rotation)

                # Combine the rotation matrix and translation vector
                T = np.identity(4)
                T[:3, :3] = R[:3, :3]
                T[:3, 3] = translation

                return T
            
             # Form the homogeneous transformation matrix
            T_camera_to_world = create_homogeneous_matrix(translation, rotation)

            # Extract the object location in the camera frame from the PointStamped message
            object_location_camera_frame = np.array(object_location_cam)
            print("object_location_camera_frame :")
            print(object_location_camera_frame)

            # Perform the transformation to get the object location in the world (or robot arm base) frame
            object_location_world_frame = np.dot(T_camera_to_world, object_location_camera_frame)
            print("object_location_world_frame :")
            print(object_location_world_frame)

            # Convert the object location to the world frame (or robot arm base frame)
            # object_location_world = tf_trans.concatenate_matrices(transformation_matrix, object_location_cam)
            # print("Object location: " )
            # print(object_location_world)

            # Create a new PointStamped message to hold the converted object location
            # object_location_world_msg = PointStamped()
            # object_location_world_msg.header.frame_id = "world"
            # object_location_world_msg.header.stamp = rospy.Time.now()
            # object_location_world_msg.point = Point(x=object_location_world[0], y=object_location_world[1], z=object_location_world[2])

            # Publish the converted object location
            # self.world_frame_pub.publish(object_location_world_msg)

        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #     rospy.logwarn(f"Failed to obtain transformation: {e}")

def main():
    converter_node = ObjectLocationConverterNode()
    rospy.spin()

if __name__ == '__main__':
    main()
