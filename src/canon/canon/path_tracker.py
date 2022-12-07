#!/usr/bin/env python3

import rclpy
import os
import csv
import sys
import math
import numpy as np

from enum import IntEnum
from typing import List
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String, Int64
from nav_msgs.msg import Odometry, Path
from lgsvl_msgs.msg import VehicleControlData
from geometry_msgs.msg import Pose, PoseArray
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

class PathTracker(Node):

    def __init__(self, racecar_ns: str) -> None:

        # ROS node init
        super().__init__(f"{racecar_ns}_path_tracker")

        # dynamic transform
        self.tf_pub = TransformBroadcaster(self)
        self.tf = TransformStamped()
        self.tf.child_frame_id = racecar_ns
        self.tf.header.frame_id = "map"

        # path server and bounds info
        self.raceline = self.read_splines_info("optimal")
        self.inner_bounds = self.read_splines_info("inner_bounds")
        self.outer_bounds = self.read_splines_info("outer_bounds")

        # localizer node variables
        self.first_loc_found = False
        self.known_loc_idx = -1
        self.dyn_range_loc = 100

        # node spinners and callbacks
        self.create_subscription(Odometry, f"/{racecar_ns}/odometry", self.path_tracker_node, 1)

    def assemble_dynamic_tf(self, state: Odometry) -> None:

        # assemble tf elements for the racecar
        pos = Vector3(x = state.pose.pose.position.x, y = state.pose.pose.position.y)
        rot = state.pose.pose.orientation
        self.tf.transform = Transform(translation = pos, rotation = rot)
        self.tf.header.stamp = self.get_clock().now().to_msg()

    def read_splines_info(self, trajectory: str) -> List:

        # read spline points from the CSV and store it in the main class
        rel_path = os.path.join(get_package_share_directory("canon"), "maps")
        trajectory = csv.reader(open(os.path.expanduser(f"{rel_path}/{trajectory}.csv")), delimiter = ",")
        return(np.array([[float(p) for p in point] for point in trajectory]))
    
    def localize_raccecar(self, state: Odometry) -> tuple(int, float):

        # decompose state vector
        x, y = state.pose.pose.position.x, state.pose.pose.position.y

        # if localizer is not initialized, search the entire trajectory
        if not self.first_loc_found:
            _ranges = []
            for i in range(len(self.raceline)):
                _xi, _yi = self.raceline[i][0], self.raceline[i][1]
                _ranges.append(math.hypot(x - _xi, y - _yi))
            _min_range = min(_ranges)
            self.first_loc_found = True
            self.known_loc_idx = _ranges.index(_min_range)
        
        # after initialization, check for points within the local area defined by self.dyn_range_loc
        else:
            _ranges = []
            for i in range(-self.dyn_range_loc, self.dyn_range_loc):
                _loc_idx = (self.known_loc_idx + i) % len(self.raceline)
                _xi, _yi = self.raceline[_loc_idx][0], self.raceline[_loc_idx][1]
                _ranges.append(math.hypot(x - _xi, y - _yi))
            _min_range = min(_ranges)
            self.known_loc_idx += _ranges.index(_min_range) - self.dyn_range_loc
            self.known_loc_idx %= len(self.raceline)
        
        return(self.known_loc_idx, _min_range)

    def path_tracker_node(self, state: Odometry) -> None:

        self.assemble_dynamic_tf(state)

        self.tf_pub.sendTransform(self.tf)