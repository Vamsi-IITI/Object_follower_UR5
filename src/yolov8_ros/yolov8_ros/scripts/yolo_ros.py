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
        self.depth_subscriber = rospy.Subscriber(depth_topic, Image, self.depth_callback)

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

    weights_path = rospy.get_param("~weights_path", "yolov8n.pt")
    classes_path = rospy.get_param("~classes_path", "$(find yolov8_ros)/model/coco.txt")
    img_topic =    rospy.get_param("~img_topic", "/camera/rgb/image_raw")
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