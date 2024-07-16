#!/usr/bin/python

import rospy
import sensor_msgs.msg
from std_srvs.srv import Empty
from cv_bridge import CvBridge
import cv2
import math
from line_tracking_race.msg._Errors import Errors

class Planner:

    def __init__(self):

        self.STOP_ERROR_PUBLISHING = False
        self.bridge = CvBridge()
        self.LOWER_YELLOW = (20, 50, 50)
        self.UPPER_YELLOW = (30, 255, 255)

        self.camera_subscriber = rospy.Subscriber(
            '/car/image_raw', 
            sensor_msgs.msg.Image,
            self.camera_callback
        )
        
        self.current_error_publisher = rospy.Publisher(
            "/planning/current_error", 
            Errors, 
            queue_size=1
        )

        rospy.wait_for_service('/gazebo/reset_simulation')
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        
        rospy.wait_for_service('/gazebo/reset_world')
        self.reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)

        rospy.loginfo('[PLANNER] NODE STARTED')


    def camera_callback(self, msg):

        if msg is None:
            return

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
            
            # base_image = cv_image.copy()
            
            cv2.line(cv_image, (camera_center[0] - 20, camera_center[1]), (camera_center[0] + 20, camera_center[1]), (0, 0, 255), 2)
            cv2.line(cv_image, (camera_center[0], camera_center[1] - 20), (camera_center[0], camera_center[1] + 20), (0, 0, 255), 2)
            
            if centroid:
                cv2.circle(cv_image, centroid, 5, (0, 255, 0), -1)
                cv2.putText(cv_image, f'Centroid: {centroid}', (10, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            
            masked_image_with_centroid = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            if centroid:
                cv2.circle(masked_image_with_centroid, centroid, 5, (0, 255, 0), -1)
                cv2.putText(masked_image_with_centroid, f'Centroid: {centroid}', (10, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

            # cv2.imshow('Processed Image', cv_image)
            # cv2.imshow('Masked Image', mask)
            # cv2.imshow('Masked Image with Centroid', masked_image_with_centroid)
            # cv2.imshow('Base Image', base_image)
            cv2.waitKey(1)

            x_axis_error, distance_error, angle_error = self.compute_current_error(centroid, camera_center, width, height)

            error_msg = Errors()
            error_msg.angle_error = angle_error
            error_msg.distance_error = distance_error
            error_msg.x_axis_error = x_axis_error

            self.current_error_publisher.publish(error_msg)
            
        except Exception as e:
            rospy.logerr(f"Error during the image processing: {e}")

    def compute_current_error(self, centroid, camera_center, width, height):
        offset = centroid[0] - camera_center[0] 
        max_offset = width / 2
        x_axis_error = (offset + max_offset) / max_offset - 1
        
        offset = centroid[0] - camera_center[0]
        distance = math.dist(centroid, camera_center)
        max_distance = width
        distance_error = (distance + max_distance) / max_distance - 1
        distance_error = distance_error if offset > 0 else -distance_error


        car_position = (math.floor(width / 2), height - 1)
        distance = math.dist(car_position, centroid)
        angle = math.asin((centroid[0] - car_position[0]) / distance)
        angle_degrees = angle * 180 / math.pi
        angle_normalized = (angle_degrees + 90) / 90 - 1
        angle_error = angle_normalized

        return x_axis_error, distance_error, angle_error

if __name__ == '__main__':
    rospy.init_node('Planner')
    planner = Planner()
    rospy.spin()
    cv2.destroyAllWindows()
    rospy.loginfo("[PLANNER] STOPPING NODE")
