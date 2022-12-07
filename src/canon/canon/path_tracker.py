#!/usr/bin/env python3

import rclpy
import os
import csv
import sys
import math

from enum import IntEnum
from typing import Tuple
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from lgsvl_msgs.msg import VehicleControlData
from geometry_msgs.msg import Pose, PoseArray
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

class SplineIdentifier(IntEnum):
    ActiveRaceline = 0
    TrackInnerBounds = 1
    TrackOuterBounds = 2

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
        raceline, raceline_res = self.read_splines_info("optimal")
        inner_bounds, inner_bounds_res = self.read_splines_info("inner_bounds")
        outer_bounds, outer_bounds_res = self.read_splines_info("outer_bounds")
        self.splines_points = (raceline, inner_bounds, outer_bounds)
        self.splines_res = (raceline_res, inner_bounds_res, outer_bounds_res)

        # localizer node variables
        self.first_loc_found = False
        self.known_loc_idx = (-1, -1, -1)
        self.dyn_range_loc = (100, 200, 200)

        # path tracker parameters
        self.wheelbase_len = 3.0
        self.time_tick = 0.01
        self.horizon_len = 10
        self.min_speed = 5.0
        self.max_speed = 40.0

        # node publishers
        self._lat_err_pub = self.create_publisher(Float64, f"/{racecar_ns}/path_tracker/lat_err", 1)
        self._lon_err_pub = self.create_publisher(Float64, f"/{racecar_ns}/path_tracker/lon_err", 1)
        self._inner_bounds_pub = self.create_publisher(PoseArray, f"/{racecar_ns}/path_server/inner_bounds", 1)
        self._outer_bounds_pub = self.create_publisher(PoseArray, f"/{racecar_ns}/path_server/outer_bounds", 1)
        self._target_ref_pub = self.create_publisher(PoseArray, f"/{racecar_ns}/path_server/target_spline", 1)

        # node spinners and callbacks
        self.create_subscription(Odometry, f"/{racecar_ns}/odometry", self.path_tracker_node, 1)

    def assemble_dynamic_tf(self, state: Odometry) -> None:

        # assemble tf elements for the racecar
        pos = Vector3(x = state.pose.pose.position.x, y = state.pose.pose.position.y)
        rot = state.pose.pose.orientation
        self.tf.transform = Transform(translation = pos, rotation = rot)
        self.tf.header.stamp = self.get_clock().now().to_msg()

    def yaw_to_quat(self, yaw: float) -> Quaternion:
            quat   = Quaternion()
            quat.z = math.sin(yaw / 2.0)
            quat.w = math.cos(yaw / 2.0)
            return(quat)

    def read_splines_info(self, trajectory: str) -> Tuple[PoseArray, float]:

        # read spline points from the CSV and store it in the main class
        rel_path = os.path.join(get_package_share_directory("canon"), "maps")
        trajectory = csv.reader(open(os.path.expanduser(f"{rel_path}/{trajectory}.csv")), delimiter = ",")
        raw_pts = [[float(point[0]), float(point[1])] for point in trajectory]

        # convert the trajectory to an array of poses
        points = PoseArray()
        for i in range(len(raw_pts)):
            pose = Pose()
            _next = (i + 1) % len(raw_pts)
            pose.position.x, pose.position.y = raw_pts[i][0], raw_pts[i][1]
            pose.orientation = self.yaw_to_quat(math.atan2(raw_pts[_next][1] - raw_pts[i][1], raw_pts[_next][0] - raw_pts[i][0]))
            points.poses.append(pose)
        
        # calculate the average spline resolution
        _res = []
        for i in range(len(points.poses)):
            _next = i, (i + 1) % len(points.poses)
            _ix = points.poses[i][0] - points.poses[_next][0]
            _iy = points.poses[i][1] - points.poses[_next][1]
            _res.append(math.hypot(_ix, _iy))
        _res = sum(_res) / len(_res)
        return(points, _res)
    
    def localize_racecar_on_spline(self, state: Odometry, target_tup_idx: int, first_loc: bool) -> Tuple[int, float]:

        # decompose state vector
        x, y = state.pose.pose.position.x, state.pose.pose.position.y

        # if the spline localizer is not initialized, search the entire spline for the first localized index
        if not first_loc:
            _target_spline_ranges = []
            for i in range(len(self.splines_points[target_tup_idx])):
                _xi, _yi = self.splines_points[target_tup_idx].poses[i].x, self.splines_points[target_tup_idx].poses[i].y
                _target_spline_ranges.append(math.hypot(x - _xi, y - _yi))
            _target_spline_lat_sep = min(_target_spline_ranges)
            _target_spline_loc_idx = _target_spline_ranges.index(_target_spline_lat_sep)
        
        # after initialization, check for points within the local area defined by self.dyn_range_loc
        else:
            _target_spline_ranges = []
            for i in range(-self.dyn_range_loc[target_tup_idx], self.dyn_range_loc[target_tup_idx]):
                _loc_idx = (self.known_loc_idx + i) % len(self.splines_points[target_tup_idx])
                _xi, _yi = self.splines_points[target_tup_idx].poses[_loc_idx].x, self.splines_points[target_tup_idx].poses[_loc_idx].y
                _target_spline_ranges.append(math.hypot(x - _xi, y - _yi))
            _target_spline_lat_sep = min(_target_spline_ranges)
            _target_spline_loc_idx = self.known_loc_idx[target_tup_idx] +_target_spline_ranges.index(_target_spline_lat_sep) 
            _target_spline_loc_idx -= self.dyn_range_loc[target_tup_idx]
            _target_spline_loc_idx %= len(self.splines_points[target_tup_idx])
        
        # return the localized spline index and the lateral seperation to the target spline
        return(_target_spline_loc_idx, _target_spline_lat_sep)

    def localize_raccecar(self, state: Odometry) -> Tuple[Tuple[int], Tuple[float]]:

        # localize the racecar for the active raceline and the track bounds
        _raceline_loc_idx, _raceline_lat_sep = self.localize_racecar_on_spline(state, int(SplineIdentifier.ActiveRaceline), self.first_loc_found)
        _inner_bounds_loc_idx, _inner_bounds_lat_sep = self.localize_racecar_on_spline(state, int(SplineIdentifier.TrackInnerBounds), self.first_loc_found)
        _outer_bounds_loc_idx, _outer_bounds_lat_sep = self.localize_racecar_on_spline(state, int(SplineIdentifier.TrackOuterBounds), self.first_loc_found)

        # localizer initialization is always set true after the first cycle,
        # unless it is reset by an outside method
        if not self.first_loc_found:
            self.first_loc_found = True
        
        # return a tuple containing the localized index and lateral error
        return((_raceline_loc_idx, _inner_bounds_loc_idx, _outer_bounds_loc_idx), (_raceline_lat_sep, _inner_bounds_lat_sep, _outer_bounds_lat_sep))

    def assemble_reference_points(self, vel: float, loc_idx: int, target_tup_idx: int, reverse_pts: bool) -> PoseArray:

        # calcluate the skip index interval based on the current racecar velocity
        idx_skip = min(int(vel * self.time_tick / self.splines_res[target_tup_idx]), 1)
        reference = PoseArray()

        # if reversed points are necessary (true, if track bounds are involved)
        if reverse_pts:
            for i in range(self.horizon_len - 1):
                curr_idx = (self.horizon_len - i - 1) * idx_skip % len(self.splines_points[target_tup_idx])
                reference.poses.append(self.splines_points[target_tup_idx].poses[curr_idx])

        # populate the evenly samped spline points
        for i in range(self.horizon_len):
            curr_idx = (i + 1) * idx_skip % len(self.splines_points[target_tup_idx])
            reference.poses.append(self.splines_points[target_tup_idx].poses[curr_idx])
        return(reference)

    def assemble_references(self, state: Odometry, _loc_idx: Tuple[int, int, int]) -> Tuple[PoseArray, PoseArray, PoseArray]:

        # calculate the current velocity and enforce bounds vioaltion
        vel = min(max(math.hypot(state.twist.twist.linear.x, state.twist.twist.linear.y), self.min_speed), self.max_speed)

        # assemble reference spline pose array for the active raceline and the track bounds
        _raceline_ref = self.assemble_reference_points(vel, _loc_idx[int(SplineIdentifier.ActiveRaceline)], int(SplineIdentifier.ActiveRaceline), False)
        _inner_bounds_ref = self.assemble_reference_points(vel, _loc_idx[int(SplineIdentifier.TrackInnerBounds)], int(SplineIdentifier.TrackInnerBounds), False)
        _outer_bounds_ref = self.assemble_reference_points(vel, _loc_idx[int(SplineIdentifier.TrackOuterBounds)], int(SplineIdentifier.TrackOuterBounds), False)
        return(_raceline_ref, _inner_bounds_ref, _outer_bounds_ref)

    def path_tracker_node(self, state: Odometry) -> None:

        # execute path tracker node tasks sequentially
        self.assemble_dynamic_tf(state)
        _loc_idx, _lat_err = self.localize_raccecar(state)
        _refs = self.assemble_references(state, _loc_idx)

        # pubish all the desired information
        self.tf_pub.sendTransform(self.tf)
        self._lat_err_pub.publish(Float64(data = _lat_err[int(SplineIdentifier.ActiveRaceline)]))
        self._inner_bounds_pub.publish(_refs[int(SplineIdentifier.TrackInnerBounds)])
        self._outer_bounds_pub.publish(_refs[int(SplineIdentifier.TrackOuterBounds)])
        self._target_ref_pub.publish(_refs[int(SplineIdentifier.ActiveRaceline)])

def path_tracker():
    rclpy.init()
    racecar_ns = str(sys.argv[1])
    node = PathTracker(racecar_ns)
    rclpy.spin(node)