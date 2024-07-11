#!/usr/bin/python

import os
from datetime import datetime
import csv

import rospy
import rospkg
import std_msgs.msg


MAX_THRUST = 15  # Maximum base wheel speed
RAMP_UP = 0.5  # Acceleration step

TURNING_THRUST = 7.5  # Speed devoted to turning


class Controller:

    def __init__(self):
        # Hard-coded PID parameters
        self.k_p = 1.0
        self.k_i = 0.01
        self.k_d = 0.1

        rospy.loginfo(f"PID params: {self.k_p}, {self.k_i}, {self.k_d}")

        self.prev_error = 0
        self.accumulated_integral = 0
        self.time_start = 0
        self.time_prev = 0

        self.thrust = 0

        self.started = False

        self.left_wheel_pub = rospy.Publisher(
            "/car/front_left_velocity_controller/command",
            std_msgs.msg.Float64,
            queue_size=10,
        )

        self.right_wheel_pub = rospy.Publisher(
            "/car/front_right_velocity_controller/command",
            std_msgs.msg.Float64,
            queue_size=10,
        )

        self.error_sub = rospy.Subscriber(
            "/planning/current-error",
            std_msgs.msg.Float32,
            self.current_error_callback,
        )

        rospy.loginfo("[CONTROLLER] NODE STARTED")

    def current_error_callback(self, msg):
        measurement = msg.data
        time_now = rospy.get_rostime()

        if time_now == 0:
            rospy.logwarn("get_rostime() returned 0. Skipping.")
            return

        if not self.started:
            self.time_start = time_now
            self.time_prev = time_now
            self.started = True
            return

        error = measurement
        dt = (time_now - self.time_prev).to_sec()

        self.accumulated_integral += dt * (error + self.prev_error) / 2

        p_term = self.k_p * error
        i_term = self.k_i * self.accumulated_integral
        d_term = self.k_d * (error - self.prev_error) / dt
        control = p_term + i_term + d_term

        self.prev_error = error
        self.time_prev = time_now

        if self.thrust < MAX_THRUST:
            self.thrust += RAMP_UP

        rospy.loginfo(f'Error: {error} ; Control {control}')

        # if control == 0 => go straight
        # if control > 0 => turn left (lwv > rwv) else turn right
        left_wheel_velocity = self.thrust + TURNING_THRUST * control
        right_wheel_velocity = self.thrust - TURNING_THRUST * control

        msg = std_msgs.msg.Float64()
        msg.data = left_wheel_velocity
        self.left_wheel_pub.publish(msg)
        
        msg.data = right_wheel_velocity
        self.right_wheel_pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("Controller")
    Controller()
    rospy.spin()
    rospy.loginfo("[CONTROLLER] STOPPING NODE")
