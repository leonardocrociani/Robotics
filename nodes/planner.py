#!/usr/bin/python

import rospy
import sensor_msgs.msg
import std_msgs.msg
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class Planner:

    def __init__(self):
        self.seen_images = []
        self.bridge = CvBridge()
        self.LOWER_YELLOW = (20, 50, 50)
        self.UPPER_YELLOW = (30, 255, 255)

        self.camera_subscriber = rospy.Subscriber(
            '/car/image_raw', 
            sensor_msgs.msg.Image,
            self.camera_callback
        )
        
        self.current_error_publisher = rospy.Publisher(
            "/planning/current-error", 
            std_msgs.msg.Float32, 
            queue_size=1
        )

        rospy.loginfo('[PLANNER] NODE STARTED')

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            height, width, _ = cv_image.shape
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, self.LOWER_YELLOW, self.UPPER_YELLOW)
            M = cv2.moments(mask)

            if M["m00"] != 0:
                centroid = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            else:
                centroid = None

            camera_center = (math.floor(width / 2), math.floor(height / 2))
            
            cv2.line(cv_image, (camera_center[0] - 20, camera_center[1]), (camera_center[0] + 20, camera_center[1]), (0, 0, 255), 2)
            cv2.line(cv_image, (camera_center[0], camera_center[1] - 20), (camera_center[0], camera_center[1] + 20), (0, 0, 255), 2)
        
            if centroid:
                cv2.circle(cv_image, centroid, 5, (0, 255, 0), -1)
                cv2.putText(cv_image, f'Centroid: {centroid}', (10, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            
            cv2.imshow('Processed Image', cv_image)
            cv2.waitKey(1)  

            current_error = self.compute_current_error(centroid, camera_center, width, height)

            error_msg = std_msgs.msg.Float32()
            error_msg.data = current_error

            self.current_error_publisher.publish(error_msg)
            
        except Exception as e:
            rospy.logerr(f"Error during the image processing: {e}")

    def compute_current_error (self, centroid, camera_center, width, height):

        # ---- Error 1: x-axis ----
        offset = centroid[0] - camera_center[0] 
        max_offset = width / 2
        current_error = (offset + max_offset) / max_offset - 1

        return current_error

if __name__ == '__main__':
    rospy.init_node('Planner')
    planner = Planner()
    rospy.spin()
    cv2.destroyAllWindows()
    rospy.loginfo("[PLANNER] STOPPING NODE")
