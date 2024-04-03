#!/usr/bin/env python

import rospy
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from jetbot_vision.msg import ObjDepths
import numpy as np
import cv2

class Depthector:
    def __init__(self):
        rospy.init_node('depthector', anonymous=True)

        self.detect_sub = rospy.Subscriber('detectnet/detections', Detection2DArray, self.detect_callback)
        self.depth_sub = rospy.Subscriber('camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)

        self.overlay_pub = rospy.Publisher('detectnet/depth_overlay', Image, queue_size=10)
        self.depths_pub = rospy.Publisher('detectnet/obj_depths', ObjDepths, queue_size=10)
        
        self.bridge = CvBridge()
        self.depth_image = None
        self.detections = None
        self.labels = self.load_labels()

    def load_labels(self):
        labels = []
        with open('/home/anaam/catkin_ws/src/jetbot/jetbot_vision/include/jetbot_vision/ssd_coco_labels.txt', 'r') as f:
            for line in f:
                labels.append(line.strip())
        return labels

    def detect_callback(self, data):
        self.detections = data

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

    def compute_average_depth(self, bbox):
        x_min = int(bbox.center.x - bbox.size_x / 2)
        y_min = int(bbox.center.y - bbox.size_y / 2)
        x_max = int(bbox.center.x + bbox.size_x / 2)
        y_max = int(bbox.center.y + bbox.size_y / 2)

        depths = self.depth_image[y_min:y_max, x_min:x_max]
        average_depth = np.mean(depths)/1000

        return average_depth

    def get_obj_depths(self):
        if self.depth_image is not None and self.detections is not None:
            try:
                depth_image_with_overlay = self.depth_image

                class_list = []
                depth_list = []
                x_list = []
                y_list = []

                for detection in self.detections.detections:
                    average_depth = self.compute_average_depth(detection.bbox)
                    class_name = self.labels[detection.results[0].id]

                    depth_list.append(average_depth)
                    class_list.append(class_name)
                    x_list.append(detection.bbox.center.x)
                    y_list.append(detection.bbox.center.y)

                    cv2.putText(depth_image_with_overlay, 'Class: {}, Depth: {:.2f} m'.format(class_name, average_depth),
                                (int(detection.bbox.center.x), int(detection.bbox.center.y - 10)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

                overlay_msg = self.bridge.cv2_to_imgmsg(depth_image_with_overlay, encoding="passthrough")
                self.overlay_pub.publish(overlay_msg)          
                
                depths_msg = ObjDepths()
                depths_msg.header = self.detections.header
                depths_msg.classes = class_list
                depths_msg.depths = depth_list
                depths_msg.centers_x = x_list
                depths_msg.centers_y = y_list
                self.depths_pub.publish(depths_msg)

            except CvBridgeError as e:
                print(e)

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            self.get_obj_depths()
            rate.sleep()


if __name__ == '__main__':
    try:
        depthector = Depthector()
        depthector.run()
    except rospy.ROSInterruptException:
        pass
