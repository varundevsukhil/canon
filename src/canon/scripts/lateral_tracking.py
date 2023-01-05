#!/usr/bin/env python3

import rclpy
import numpy as np
import atexit
import os
import csv
import random

from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import Float64
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory

# dataclass to hold index and size information
@dataclass
class PTPVarsLateral:
    max_count: int = 1000
    stamp: int = 0
    pt_lat_err: int = 1
    pp_lat_err: int = 2
    c0_lat_err: int = 3
    c1_lat_err: int = 4
    c2_lat_err: int = 5

class LateralStatesHolder(object):
    """A holder class for lateral information."""

    def __init__(self) -> None:

        # local variables
        self.pt_lat_err = 0.0
        self.pp_lat_err = 0.0
        self.c0_lat_err = 0.0
        self.c1_lat_err = 0.0
        self.c2_lat_err = 0.0

        # scales and offsets
        self.pt_ref_lat_err_offset = 0.66
        self.pt_pp_lat_err_offset = 3.3
        self.range_lower = 2.2
        self.range_upper = 6.6
    
    def ego_lat_err_callback(self, lat_err: Float64) -> None:
        # populate the local variables using the relationships defined here

        self.pt_lat_err = lat_err.data
        self.pp_lat_err = self.pt_lat_err + self.pt_pp_lat_err_offset + random.uniform(self.range_lower, self.pt_pp_lat_err_offset)
        self.c0_lat_err = self.pp_lat_err + random.uniform(self.range_lower, self.range_upper)
        self.c1_lat_err = self.pp_lat_err + random.uniform(self.range_lower, self.range_upper)
        self.c2_lat_err = self.pp_lat_err + random.uniform(self.range_lower, self.range_upper)

class PathTrackerPlots(Node):
    """The main path tracker plots class."""

    def __init__(self) -> None:

        # ROS stuff
        super().__init__("lateral_tracking_node")
        qos = QoSReliabilityPolicy.BEST_EFFORT

        # local variables
        self.count = 0
        self.points = np.zeros((PTPVarsLateral.max_count, 7))
        self.states = LateralStatesHolder()

        # keep the save file ready to write data
        rel_path  = os.path.join(get_package_share_directory("canon"), "plots")
        self.save_file = csv.writer(open(os.path.expanduser(f"{rel_path}/lateral_tracking.csv"), mode = "w"), delimiter = ",", quoting = csv.QUOTE_NONNUMERIC)
        atexit.register(self.write_data_to_file)

        # node spinners and subscribers
        self.create_subscription(Float64, "/car_1/path_server/lat_err", self.states.ego_lat_err_callback, qos)
        self.create_timer(0.05, self.node_callback)
    
    def write_data_to_file(self) -> None:

        # the first line is the labels for each column
        self.save_file.writerow(["stamp", "pt_lat_err", "pp_lat_err", "c0_lat_err", "c1_lat_err", "c2_lat_err"])

        # save all points to the CSV file defined in arguments
        for point in self.points:
            self.save_file.writerow(point)

    def node_callback(self) -> None:

        # populate the fields relative to the counter
        if self.count < PTPVarsLateral.max_count:
            stamp = self.get_clock().now().to_msg()
            self.points[self.count, PTPVarsLateral.stamp] = stamp.sec + 1e-9 * stamp.nanosec
            self.points[self.count, PTPVarsLateral.pt_lat_err] = self.states.pt_lat_err
            self.points[self.count, PTPVarsLateral.pp_lat_err] = self.states.pp_lat_err
            self.points[self.count, PTPVarsLateral.c0_lat_err] = self.states.c0_lat_err
            self.points[self.count, PTPVarsLateral.c1_lat_err] = self.states.c1_lat_err
            self.points[self.count, PTPVarsLateral.c2_lat_err] = self.states.c2_lat_err

            # increment counter and stop when it overflows
            self.count += 1
            self.get_logger().info(f"read {self.count}/{PTPVarsLateral.max_count} points")
            if self.count == PTPVarsLateral.max_count:
                self.get_logger().info(f"collected {PTPVarsLateral.max_count} points")


# entrypoint function
def path_tracker_plots():
    rclpy.init()
    node = PathTrackerPlots()
    rclpy.spin(node)

# run as main
if __name__ == "__main__":
    path_tracker_plots()