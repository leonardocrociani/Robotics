#!/usr/bin/python

import rospy
import std_msgs.msg
import cv2
from line_tracking_race.msg._Errors import Errors

class Monitor:

    def __init__(self):
        self.error_series = []
        self.time_start = 0
        self.time_prev = 0
        self.started = False

        self.error_type = rospy.get_param('~error_type', 'x-axis')
        self.Kp = rospy.get_param("~Kp", 1.0)
        self.Ki = rospy.get_param("~Ki", 0.01)
        self.Kd = rospy.get_param("~Kd", 0.1)

        self.camera_subscriber = rospy.Subscriber(
            '/planning/current_error', 
            Errors,
            self.error_callback,
        )

        rospy.on_shutdown(self.shutdown_callback)
        rospy.loginfo("[MONITOR] NODE STARTED")

        
    def error_callback(self, msg):
        time_now = rospy.get_rostime()

        if time_now == 0:
            return

        if not self.started:
            self.time_start = time_now
            self.time_prev = time_now
            self.started = True
            return

        dt = (time_now - self.time_prev).to_sec()

        if dt == 0:
            return
        
        error = None

        if self.error_type == 'x-axis':
            error = msg.x_axis_error
        elif self.error_type == 'distance':
            error = msg.distance_error
        elif self.error_type == 'angle':
            error = msg.angle_error

        self.error_series.append((error,dt,time_now))

    def shutdown_callback(self):
        rospy.loginfo("[MONITOR] STOPPING NODE")
        filename = f'/home/leonardo/catkin_ws/src/line_tracking_race/performances/{self.Kp}-{self.Ki}-{self.Kd}-{self.error_type}.csv'
        with open(filename, 'w') as file:
            file.write('\n'.join(map(str, self.error_series)))
        rospy.loginfo(f"[MONITOR] Error data saved to {filename}")

if __name__ == '__main__':
    rospy.init_node('Monitor')
    planner = Monitor()
    rospy.spin()
    cv2.destroyAllWindows()