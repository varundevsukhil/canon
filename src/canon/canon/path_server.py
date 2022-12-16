#!/usr/bin/env python

import rclpy
import os
import sys
import csv
import math
import numpy as np

from typing import Tuple
from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
from std_msgs.msg import UInt8, Float64
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion
from nav_msgs.msg import Odometry
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory

# trajectory enum codes
@dataclass
class PSCode:
    pitlane: int = 0
    optimal: int = 1
    overtake_dynamic: int = 2
    defense_dynamic: int = 3

# path server node parameters
# TODO: maybe move this to ROS param server?
@dataclass
class PSParams:
    fixed_search_range: int = 100
    time_tick: float = 0.4
    horizon_len: int = 10
    min_speed: float = 5.0
    max_speed: float = 15.0

# each point on the spline is a pose
# we only care about {x, y, theta}, but ROS requires Quaternion
# this dataclass holds the indices for the conversion
@dataclass
class PSIndex:
    x: int = 0
    y: int = 1
    rot_z: int = 2
    rot_w: int = 3

class PathServer(Node):
    """
    Send a pose array of references after localizing the racecar on the target spline.
    """

    def __init__(self, racecar_ns: str) -> None:

        # ROS initialization
        super().__init__(f"{racecar_ns}_path_server")

        # the static spline resources
        _pitlane, _pitlane_res = self.read_splines_info("pitlane_centerline")
        _optimal, _optimal_res = self.read_splines_info("optimal")
        self.spline_points = (_pitlane, _optimal)
        self.spline_res = (_pitlane_res, _optimal_res)

        # localizer node variables
        self.active_spline = PSCode.pitlane
        self.loc_index = -1
        self.loc_found = False

        # node publishers
        self.pt_spline = PoseArray()
        self.pt_spline.header.frame_id = "map"
        self.lat_err_pub = self.create_publisher(Float64, f"/{racecar_ns}/path_server/lat_err", QoSReliabilityPolicy.BEST_EFFORT)
        self.spline_pub = self.create_publisher(PoseArray, f"/{racecar_ns}/path_server/spline", QoSReliabilityPolicy.BEST_EFFORT)

        # node subscribers and spinners
        self.create_subscription(Odometry, f"/{racecar_ns}/odometry", self.path_server_node, QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(UInt8, f"/{racecar_ns}/argos/spline_code", self.update_target_spline, QoSReliabilityPolicy.BEST_EFFORT)
    
    def update_target_spline(self, target_code: UInt8) -> None:

        # update the active spline and reset localization whenever a new code is sent
        if target_code.data != self.active_spline:
            self.active_spline = target_code.data
            self.loc_found = False
    
    def yaw_to_quaternion(self, yaw: float) -> Tuple[float, float]:

        # convert yaw to quaternion; ignore pitch and roll
        _quat_z = math.sin(yaw / 2.0)
        _quat_w = math.cos(yaw / 2.0)
        return(_quat_z, _quat_w)

    def read_splines_info(self, name: str) -> Tuple[np.ndarray, float]:

        # read spline points from the CSV and store it in the main class
        rel_path = os.path.join(get_package_share_directory("canon"), "maps")
        raw_pts = csv.reader(open(os.path.expanduser(f"{rel_path}/{name}.csv")), delimiter = ",")
        raw_pts = [[float(point[0]), float(point[1])] for point in raw_pts]

        # convert the trajectory to an np.array of poses
        _points = np.empty([len(raw_pts), 4])
        for i in range(len(raw_pts)):
            _next = (i + 1) % len(raw_pts)
            _x, _y = raw_pts[i][PSIndex.x], raw_pts[i][PSIndex.y]
            _nx, _ny = raw_pts[_next][PSIndex.x], raw_pts[_next][PSIndex.y]
            _yaw = math.atan2(_ny - _y, _nx - _x)
            _qz, _qw = self.yaw_to_quaternion(_yaw)
            _points[i] = np.array([_x, _y, _qz, _qw])

        # calculate the average spline resolution
        _res = np.empty(len(_points) - 1)
        for i in range(len(_points) - 1):
            _next = (i + 1) % len(_points)
            _eucl_a = _points[i][PSIndex.x] - _points[_next][PSIndex.x]
            _eucl_b = _points[i][PSIndex.y] - _points[_next][PSIndex.y]
            _res[i] = math.hypot(_eucl_a, _eucl_b)
        _res = sum(_res) / len(_res)
        self.get_logger().info(f"extracted {len(_points)} from {name} points with avg. {_res} m res.")
        return(_points, _res)

    def path_server_node(self, odom: Odometry) -> None:

        # if the racecar is not localized, search the entire trajectory
        if not self.loc_found:
            _ranges = []
            for i in range(len(self.spline_points[self.active_spline])):
                _point = self.spline_points[self.active_spline][i]
                _eucl_a = odom.pose.pose.position.x - _point[PSIndex.x]
                _eucl_b = odom.pose.pose.position.y - _point[PSIndex.y]
                _ranges.append(math.hypot(_eucl_a, _eucl_b))
            _lat_err = min(_ranges)
            self.loc_found = True
            self.loc_index = _ranges.index(_lat_err)
        
        # otherwise, search only a fixed range from the previous localization index
        else:
            _ranges = []
            for i in range(-PSParams.fixed_search_range, PSParams.fixed_search_range):
                _search_idx = (self.loc_index + i) % len(self.spline_points[self.active_spline])
                _point = self.spline_points[self.active_spline][_search_idx]
                _eucl_a = odom.pose.pose.position.x - _point[PSIndex.x]
                _eucl_b = odom.pose.pose.position.y - _point[PSIndex.y]
                _ranges.append(math.hypot(_eucl_a, _eucl_b))
            _lat_err = min(_ranges)
            _loc_index = self.loc_index + _ranges.index(_lat_err) - PSParams.fixed_search_range
            self.loc_index = _loc_index % len(self.spline_points[self.active_spline])
        
        # calculate the current velocity and enforce bounds vioaltion
        # then, calculate the skip index value based on the current velocity
        vel = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)
        vel = min(max(vel, PSParams.min_speed), PSParams.max_speed)
        skip = int(vel * PSParams.time_tick * self.spline_res[self.active_spline])

        # reset the reference poses and populate new references
        self.pt_spline.poses = []
        for i in range(PSParams.horizon_len):
            _pose = Pose()
            _t_idx = (i + 1) * skip + self.loc_index
            _point = self.spline_points[self.active_spline][_t_idx]
            _pose.position = Point(x = _point[PSIndex.x], y = _point[PSIndex.y])
            _pose.orientation = Quaternion(z = _point[PSIndex.rot_z], w = _point[PSIndex.rot_w])
            self.pt_spline.poses.append(_pose)

        # send reference and lat_sep data to path_tracker
        self.spline_pub.publish(self.pt_spline)
        self.lat_err_pub.publish(Float64(data = _lat_err))

def path_server():
    rclpy.init()
    racecar_ns = str(sys.argv[1])
    node = PathServer(racecar_ns)
    rclpy.spin(node)

if __name__ == "__main__":
    path_server()