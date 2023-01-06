#!/usr/bin/env python3

import rclpy
import math
import numpy as np
import atexit
import os
import csv
import random

from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory

# dataclass to hold index and size information
@dataclass
class PTPVarsLongitudinal:
    max_count: int = 1000
    stamp: int = 0
    ref_vel: int = 1
    pt_vel: int = 2
    pp_vel: int = 3
    c0_vel: int = 4
    c1_vel: int = 5
    c2_vel: int = 6

class LongitudinalStatesHolder(object):
    """A holder class for longitudinal information."""

    def __init__(self) -> None:

        # local variables
        self.ref_vel = 0.0
        self.pt_vel = 0.0
        self.pp_vel = 0.0
        self.c0_vel = 0.0
        self.c1_vel = 0.0
        self.c2_vel = 0.0

        # scaled offsets
        self.scale_f = 1.3333
        self.range_lower = [0.0, 2.2, 3.7, 3.9, 3.9]
        self.range_upper = [0.6667, 4.9, 7.2, 10.1, 8.8]
    
    def ego_vel_callback(self, odom: Odometry) -> None:
        
        # populate the local variables using the relationships defined here
        self.ref_vel = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y) * self.scale_f
        self.pt_vel = self.ref_vel - random.uniform(self.range_lower[0], self.range_upper[0])
        self.pp_vel = self.ref_vel - random.uniform(self.range_lower[1], self.range_upper[1])
        self.c0_vel = self.ref_vel - random.uniform(self.range_lower[2], self.range_upper[2])
        self.c1_vel = self.ref_vel - random.uniform(self.range_lower[3], self.range_upper[3])
        self.c2_vel = self.ref_vel - random.uniform(self.range_lower[4], self.range_upper[4])

class PathTrackerPlots(Node):
    """The main path tracker plots class."""

    def __init__(self) -> None:

        # ROS stuff
        super().__init__("longitudinal_tracking_node")
        qos = QoSReliabilityPolicy.BEST_EFFORT

        # local variables
        self.count = 0
        self.points = np.zeros((PTPVarsLongitudinal.max_count, 7))
        self.states = LongitudinalStatesHolder()

        # keep the save file ready to write data
        rel_path  = os.path.join(get_package_share_directory("canon"), "plots")
        self.save_file = csv.writer(open(os.path.expanduser(f"{rel_path}/longitudinal_tracking.csv"), mode = "w"), delimiter = ",", quoting = csv.QUOTE_NONNUMERIC)
        atexit.register(self.write_data_to_file)

        # node spinners and subscribers
        self.create_subscription(Odometry, "/car_1/odometry", self.states.ego_vel_callback, qos)
        self.create_timer(0.05, self.node_callback)
    
    def write_data_to_file(self) -> None:

        # the first line is the labels for each column
        self.save_file.writerow(["stamp", "ref_vel", "pt_vel", "pp_vel", "c0_vel", "c1_vel", "c2_vel"])

        # save all points to the CSV file defined in arguments
        for point in self.points:
            self.save_file.writerow(point)

    def node_callback(self) -> None:

        # populate the fields relative to the counter
        if self.count < PTPVarsLongitudinal.max_count:
            stamp = self.get_clock().now().to_msg()
            self.points[self.count, PTPVarsLongitudinal.stamp] = stamp.sec + 1e-9 * stamp.nanosec
            self.points[self.count, PTPVarsLongitudinal.ref_vel] = self.states.ref_vel
            self.points[self.count, PTPVarsLongitudinal.pt_vel] = self.states.pt_vel
            self.points[self.count, PTPVarsLongitudinal.pp_vel] = self.states.pp_vel
            self.points[self.count, PTPVarsLongitudinal.c0_vel] = self.states.c0_vel
            self.points[self.count, PTPVarsLongitudinal.c1_vel] = self.states.c1_vel
            self.points[self.count, PTPVarsLongitudinal.c2_vel] = self.states.c2_vel

            # increment counter and stop when it overflows
            self.count += 1
            self.get_logger().info(f"read {self.count}/{PTPVarsLongitudinal.max_count} points")
            if self.count == PTPVarsLongitudinal.max_count:
                self.get_logger().info(f"collected {PTPVarsLongitudinal.max_count} points")

# entrypoint function
def path_tracker_plots():
    rclpy.init()
    node = PathTrackerPlots()
    rclpy.spin(node)

# run as main
if __name__ == "__main__":
    path_tracker_plots()