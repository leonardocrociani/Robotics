#!/usr/bin/python

import rospy
import std_msgs.msg
from line_tracking_race.msg._Errors import Errors

MAX_THRUST = 15
THRUST_STEP = 0.5
TURNING_THRUST = 7.5

class Controller:

    def __init__(self):

        self.Kp = rospy.get_param("~Kp", 1.0)
        self.Ki = rospy.get_param("~Ki", 0.01)
        self.Kd = rospy.get_param("~Kd", 0.1)

        self.error_type = rospy.get_param('~error_type', 'x-axis')

        rospy.loginfo(f"[CONTROLLER] PID params: {self.Kp}, {self.Ki}, {self.Kd} > | Error type: {self.error_type}")

        self.prev_error = 0
        self.accumulated_integral = 0
        self.time_start = 0
        self.time_prev = 0
        self.thrust = 0
        self.started = False

        self.left_wheel_publisher = rospy.Publisher(
            "/car/front_left_velocity_controller/command",
            std_msgs.msg.Float64,
            queue_size=10,
        )
        
        self.right_wheel_publisher = rospy.Publisher(
            "/car/front_right_velocity_controller/command",
            std_msgs.msg.Float64,
            queue_size=10,
        )

        self.error_subscriber = rospy.Subscriber(
            "/planning/current_error",
            Errors,
            self.current_error_callback,
        )

        rospy.loginfo("[CONTROLLER] NODE STARTED")

    def current_error_callback(self, msg):
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

        self.accumulated_integral += error * dt

        p_term = self.Kp * error
        i_term = self.Ki * self.accumulated_integral
        d_term = self.Kd * (error - self.prev_error) / dt
        pid = p_term + i_term + d_term

        self.prev_error = error
        self.time_prev = time_now

        if self.thrust < MAX_THRUST:
            self.thrust += THRUST_STEP

        left_wheel_speed = self.thrust + TURNING_THRUST * pid
        right_wheel_speed = self.thrust - TURNING_THRUST * pid

        msg = std_msgs.msg.Float64()
        msg.data = left_wheel_speed
        self.left_wheel_publisher.publish(msg)
        
        msg = std_msgs.msg.Float64()
        msg.data = right_wheel_speed
        self.right_wheel_publisher.publish(msg)

if __name__ == "__main__":
    rospy.init_node("Controller")
    Controller()
    rospy.spin()
    rospy.loginfo("[CONTROLLER] STOPPING NODE")
