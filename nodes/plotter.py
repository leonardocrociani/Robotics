#!/usr/bin/python

import os
from datetime import datetime
import csv

import rospy
import rospkg
import std_msgs.msg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Plotter:

    def __init__(self):
        self.started = False
        self.error_data = []
        self.error_sub = rospy.Subscriber(
            "/planning/current-error",
            std_msgs.msg.Float32,
            self.current_error_callback,
        )
        rospy.loginfo("[PLOTTER] NODE STARTED")
        self.init_plot()

    def current_error_callback(self, msg):
        error = msg.data
        time_now = rospy.get_rostime()
        if not self.started:
            self.started = True
            self.time_start = time_now
        self.error_data.append((time_now.to_sec() - self.time_start.to_sec(), error))

    def init_plot(self):
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'r-')
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel('Error')

        self.ani = FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot_data, interval=100)
        plt.show()

    def init_plot_data(self):
        self.line.set_data([], [])
        return self.line,

    def update_plot(self, frame):
        times, errors = zip(*self.error_data) if self.error_data else ([], [])
        self.line.set_data(times, errors)
        self.ax.set_xlim(0, max(10, times[-1]) if times else 10)
        return self.line,

if __name__ == "__main__":
    rospy.init_node("Plotter")
    Plotter()
    rospy.spin()
    rospy.loginfo("[Plotter] STOPPING NODE")
