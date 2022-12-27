#!/usr/bin/env python3

import rclpy
import os
import sys
import csv
import math
import numpy as np

from typing import Tuple
from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from nav_msgs.msg import Odometry
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory

# trajectory enum codes
@dataclass
class VCode:
    inside: int = 0
    outside: int = 1

# each point on the spline is a pose
# we only care about {x, y, theta}, but ROS requires Quaternion
# this dataclass holds the indices for the conversion
@dataclass
class VIndex:
    x: int = 0
    y: int = 1
    rot_z: int = 2
    rot_w: int = 3

# visualizer node parameters
# TODO: maybe move this to ROS param server?
@dataclass
class VParams:
    fixed_search_range: int = 100
    time_tick: float = 0.4
    horizon_len: int = 10
    pt_node_speed: float = 5.0
    lat_err_damp_f: float = 5.0
    max_lat_err_f: int = 12

class Bounds(object):

    def __init__(self, bound_ns: str) -> None:

        self.bounds, self.bounds_res = self.read_splines_info(bound_ns)
        self.loc_found = False
        self.points = PoseArray()
        self.points.header.frame_id = "map"
    
    def yaw_to_quaternion(self, yaw: float) -> Tuple[float, float]:

        # convert yaw to quaternion; ignore pitch and roll
        _quat_z = math.sin(yaw / 2.0)
        _quat_w = math.cos(yaw / 2.0)
        return(_quat_z, _quat_w)

    def read_splines_info(self, spline_name: str) -> Tuple[np.ndarray, float]:

        # read spline points from the CSV and store it in the main class
        rel_path = os.path.join(get_package_share_directory("canon"), "maps")
        raw_pts = csv.reader(open(os.path.expanduser(f"{rel_path}/{spline_name}.csv")), delimiter = ",")
        raw_pts = [[float(point[0]), float(point[1])] for point in raw_pts]

        # convert the trajectory to an np.array of poses
        _points = np.empty([len(raw_pts), 4])
        for i in range(len(raw_pts)):
            _next = (i + 1) % len(raw_pts)
            _x, _y = raw_pts[i][VIndex.x], raw_pts[i][VIndex.y]
            _nx, _ny = raw_pts[_next][VIndex.x], raw_pts[_next][VIndex.y]
            _yaw = math.atan2(_ny - _y, _nx - _x)
            _qz, _qw = self.yaw_to_quaternion(_yaw)
            _points[i] = np.array([_x, _y, _qz, _qw])

        # calculate the average spline resolution
        _res = np.empty(len(_points) - 1)
        for i in range(len(_points) - 1):
            _next = (i + 1) % len(_points)
            _eucl_a = _points[i][VIndex.x] - _points[_next][VIndex.x]
            _eucl_b = _points[i][VIndex.y] - _points[_next][VIndex.y]
            _res[i] = math.hypot(_eucl_a, _eucl_b)
        _res = sum(_res) / len(_res)
        return(_points, _res)

    def visualize_bounds(self, odom: Odometry) -> None:

        # if the racecar is not localized, search the entire trajectory
        if not self.loc_found:
            _ranges = []
            for i in range(len(self.bounds)):
                _point = self.bounds[i]
                _eucl_a = odom.pose.pose.position.x - _point[VIndex.x]
                _eucl_b = odom.pose.pose.position.y - _point[VIndex.y]
                _ranges.append(math.hypot(_eucl_a, _eucl_b))
            _lat_err = min(_ranges)
            self.loc_found = True
            self.loc_index = _ranges.index(_lat_err)
        
        # otherwise, search only a fixed range from the previous localization index
        else:
            _ranges = []
            for i in range(-VParams.fixed_search_range, VParams.fixed_search_range):
                _search_idx = (self.loc_index + i) % len(self.bounds)
                _point = self.bounds[_search_idx]
                _eucl_a = odom.pose.pose.position.x - _point[VIndex.x]
                _eucl_b = odom.pose.pose.position.y - _point[VIndex.y]
                _ranges.append(math.hypot(_eucl_a, _eucl_b))
            _lat_err = min(_ranges)
            _loc_index = self.loc_index + _ranges.index(_lat_err) - VParams.fixed_search_range
            self.loc_index = _loc_index % len(self.bounds)
        
        # calculate the skip index value based on the current velocity
        skip = int(VParams.pt_node_speed * VParams.time_tick / int(self.bounds_res))

        # reset the reference poses and populate new references
        self.points.poses = []
        _lat_sep_f = min(max(int(_lat_err * VParams.lat_err_damp_f), 1), VParams.max_lat_err_f)
        for i in range(-VParams.horizon_len, VParams.horizon_len):
            _pose = Pose()
            _t_idx = ((i + 1) * (skip + _lat_sep_f) + self.loc_index) % len(self.bounds)
            _point = self.bounds[_t_idx]
            _pose.position = Point(x = _point[VIndex.x], y = _point[VIndex.y])
            _pose.orientation = Quaternion(z = _point[VIndex.rot_z], w = _point[VIndex.rot_w])
            self.points.poses.append(_pose)

class Visualize(Node):

    def __init__(self, car_ns: str) -> None:

        super().__init__("visualize")
        qos = QoSReliabilityPolicy.BEST_EFFORT

        self.inside_bounds = Bounds("inside_bounds")
        self.outside_bounds = Bounds("outside_bounds")

        self.inside_bounds_pub = self.create_publisher(PoseArray, f"/visualize/inside_bounds", qos)
        self.outside_bounds_pub = self.create_publisher(PoseArray, f"/visualize/outside_bounds", qos)

        self.create_subscription(Odometry, f"/{car_ns}/odometry", self.inside_bounds.visualize_bounds, qos)
        self.create_subscription(Odometry, f"/{car_ns}/odometry", self.outside_bounds.visualize_bounds, qos)
        self.create_timer(0.05, self.visualize_node)
    
    def visualize_node(self) -> None:

        self.inside_bounds_pub.publish(self.inside_bounds.points)
        self.outside_bounds_pub.publish(self.outside_bounds.points)


def visualize():
    rclpy.init()
    car_ns = str(sys.argv[1])
    node = Visualize(car_ns)
    rclpy.spin(node)

if __name__ == "__main__":
    visualize()