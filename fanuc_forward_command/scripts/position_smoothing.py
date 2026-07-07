#!/usr/bin/env python3

"""
Rclpy Node to read in raw fpc commands and smooth the position commands to avoid jerky motion.
"""

from collections import deque

import rclpy
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class PositionSmoothing(Node):
    def __init__(self):
        super().__init__('position_smoothing')

        self.loop_rate = 125.0  # Hz
        self.dt = 1.0 / self.loop_rate
        self.stale_timeout = 0.2  # seconds
        self.command_pos = None
        self.last_msg_time = None
        self.last_command_pos = None

        self.pos_gain = 0.8  # Proportional gain for smoothing
        self.max_diff = 0.1  # (rad) Maximum allowed change in position per update

        self.sub = self.create_subscription(Float64MultiArray, '/forward_position_controller/commands_raw', self.fpc_callback_1, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)

        self.timer = self.create_timer(self.dt, self.timer_callback_1)  # 125 Hz

    def fpc_callback_1(self, msg):
        self.last_msg_time = self.get_clock().now().nanoseconds * 1e-9
        self.command_pos = list(msg.data)

    
    def timer_callback_1(self):
        """"Main loop to smooth the position commands and publish them."""

        if (self.command_pos is None):
            return

        cur_time = self.get_clock().now().nanoseconds * 1e-9

        if (cur_time - self.last_msg_time) > self.stale_timeout:
            # No new commands received or commands are stale, do not publish anything
            return
        
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

        # publish 
        msg = Float64MultiArray()
        msg.data = self.last_command_pos
        self.pub.publish(msg)



def main():
    rclpy.init()
    node = PositionSmoothing()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
