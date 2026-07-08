#!/usr/bin/env python3

"""
Rclpy Node to read in raw fpc commands and smooth the position commands to avoid jerky motion.
"""

from collections import deque

import csv

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from ruckig import InputParameter, OutputParameter, Result, Ruckig

class PositionSmoothing(Node):
    def __init__(self):
        super().__init__('position_smoothing')

        fp = open("/home/mowito/fanuc_ws/csv_temp_logs.csv", "w")
        self.writer = csv.writer(fp)

        self.loop_rate = 125.0  # Hz
        self.dt = 1.0 / self.loop_rate
        self.stale_timeout = 0.2  # seconds
        self.command_pos = None
        self.last_msg_time = None
        self.last_command_pos = None

        # js - params
        self.last_js_received = None
        self.joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]
        self.current_joint_position = None
        self.current_joint_velocity = None

        # ruckig params
        self.ruckig = Ruckig(6, self.dt)
        self.input = InputParameter(6)
        self.output = OutputParameter(6)
        self.temp_flag = True

        self.input.max_velocity = [2.0, 2.0, 3.0, 3.0, 3.0, 3.0]  # (rad/s) Maximum joint velocities
        self.input.max_acceleration = [4.5, 4.5, 7.0, 7.0, 7.0, 7.0]  # (rad/s^2) Maximum joint accelerations
        self.input.max_jerk = [20.0, 20.0, 30.0, 30.0, 30.0, 30.0]  # (rad/s^3) Maximum joint jerks

        self.pos_gain = 0.8  # Proportional gain for smoothing
        self.max_diff = 0.1  # (rad) Maximum allowed change in position per update

        self.sub = self.create_subscription(Float64MultiArray, '/forward_position_controller/commands_raw', self.fpc_callback_1, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.sub_1 = self.create_subscription(JointState, "/joint_states", self.js_cb, 10)

        self.timer = self.create_timer(self.dt, self.timer_callback_1)  # 125 Hz
    
    def order_correction(self, j_names, array):
        temp = [0, 0, 0, 0, 0, 0]
        for idx in range(6):
            index = int(j_names[idx][-1]) - 1       # zero indexed
            temp[index] = array[idx]
        return temp

    def js_cb(self, msg):
        temp_jn = msg.name
        temp_jp = msg.position
        temp_jv = msg.velocity

        self.current_joint_position = self.order_correction(temp_jn, temp_jp)
        self.current_joint_velocity = self.order_correction(temp_jn, temp_jv)

        self.last_js_received = self.get_clock().now().nanoseconds * 1e-9

    def fpc_callback_1(self, msg):
        self.last_msg_time = self.get_clock().now().nanoseconds * 1e-9
        self.command_pos = list(msg.data)

    
    def timer_callback_1(self):
        """"Main loop to smooth the position commands and publish them."""

        if (self.command_pos is None):
            return
        
        if self.current_joint_position is None or self.current_joint_velocity is None:
            return  # Wait until we have received joint states

        cur_time = self.get_clock().now().nanoseconds * 1e-9

        if (cur_time - self.last_msg_time) > self.stale_timeout:
            # No new commands received or commands are stale, do not publish anything
            return
        
        if (cur_time - self.last_js_received) > self.stale_timeout:
            # No new joint states received or joint states are stale, do not publish anything
            return

        """
        if (self.last_command_pos is None):
            self.last_command_pos = self.command_pos.copy()

        self.last_command_pos = np.array(self.last_command_pos)
        self.command_pos = np.array(self.command_pos)

        diff = self.command_pos - self.last_command_pos
        # Limit the maximum change in position per update
        diff = np.clip(diff, -self.max_diff, self.max_diff)

        smoothed_diff = self.pos_gain * diff
        smoothed_pos = self.last_command_pos + smoothed_diff

        self.last_command_pos = smoothed_pos.tolist()
        """

        # Use Ruckig to smooth the position commands
        if self.temp_flag:
            self.input.current_position = self.current_joint_position
            self.input.current_velocity = self.current_joint_velocity
            self.input.current_acceleration = self.output.new_acceleration
            self.temp_flag = False

        self.input.target_position = self.command_pos
        self.input.target_velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.input.target_acceleration = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        result = self.ruckig.update(self.input, self.output)
        target_command = self.output.new_position
        
        if (result != Result.Working):
            return

        self.writer.writerow(target_command)
        self.output.pass_to_input(self.input)

        # publish 
        msg = Float64MultiArray()
        msg.data = list(target_command)
        self.pub.publish(msg)



def main():
    rclpy.init()
    node = PositionSmoothing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
