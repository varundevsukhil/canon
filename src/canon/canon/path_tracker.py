#!/usr/bin/env python3

import rclpy
import os
import csv
import sys
import math

from dataclasses import dataclass
from typing import Tuple
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from lgsvl_msgs.msg import VehicleControlData
from geometry_msgs.msg import Pose, PoseArray
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory

@dataclass
class PTConstants:

    # spline identifiers
    OptimalRaceline: int = 0
    TrackInnerBounds: int = 1
    TrackOuterBounds: int = 2

    # spline adjusted velocity
    OptimalRacelineVelocity: float = 70.0
    PitlaneVelocity: float = 15.0

    # pitlane spline switch triggers
    PitlaneExitDistanceTrigger: float = 10.0
    PitlaneEntryDistanceTrigger: float = 50.0

@dataclass
class LongitudinalGains:

    # PID gains for the longitudinal controller
    P_gain: float = 0.15
    I_gain: float = 0.001
    D_gain: float = 0.015

class PathTracker(Node):
    """
    Kinematic bicycle model based MPC path tracker.
    """
    
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
        inner_bounds, inner_bounds_res = self.read_splines_info("inside_bounds")
        outer_bounds, outer_bounds_res = self.read_splines_info("outside_bounds")
        self.splines_points = (raceline, inner_bounds, outer_bounds)
        self.splines_res = (raceline_res, inner_bounds_res, outer_bounds_res)

        # localizer node variables
        self.first_loc_found = False
        self.known_loc_idx = (-1, -1, -1)
        self.dyn_range_loc = (100, 200, 200)

        # lateral control parameters
        self.steer_max = 1.0
        steer_max = 2.0
        gradient = 0.005
        candidates = [candidate * gradient for candidate in range(int(steer_max / gradient))]
        negatives = [-candidate for candidate in candidates[1:]]
        self.candidates = [math.radians(candidate) for candidate in negatives[::-1] + candidates]

        # longitudinal control parameters
        self.power_max = 1.0
        self.brake_max = 1.0
        self.prev_error = 0.0
        self.steady_err = [0.0] * 50
        self.coast_vel = 5.0

        # path tracker parameters
        self.wheelbase_len = 3.0
        self.time_tick = 0.25
        self.horizon_len = 20
        self.min_speed = 5.0
        self.max_speed = 40.0

        # persistent vehicle command and actuator rate limiter
        self.actuation_rate = 0.0025
        self.command = VehicleControlData()

        # node publishers
        self.lat_err_pub = self.create_publisher(Float64, f"/{racecar_ns}/path_tracker/lat_err", 1)
        self.lon_err_pub = self.create_publisher(Float64, f"/{racecar_ns}/path_tracker/lon_err", 1)
        self.inner_bounds_pub = self.create_publisher(PoseArray, f"/{racecar_ns}/path_server/inner_bounds", 1)
        self.outer_bounds_pub = self.create_publisher(PoseArray, f"/{racecar_ns}/path_server/outer_bounds", 1)
        self.target_ref_pub = self.create_publisher(PoseArray, f"/{racecar_ns}/path_server/target_spline", 1)
        self.optimal_pred_pub = self.create_publisher(PoseArray, f"/{racecar_ns}/path_tracker/optimal_prediction", 1)
        self.command_pub = self.create_publisher(VehicleControlData, f"/{racecar_ns}/command", 1)

        # node spinners and callbacks
        self.create_subscription(Odometry, f"/{racecar_ns}/odometry", self.path_tracker_node, 1)

    def assemble_dynamic_tf(self, state: Odometry) -> None:

        # assemble tf elements for the racecar
        pos = Vector3(x = state.pose.pose.position.x, y = state.pose.pose.position.y)
        rot = state.pose.pose.orientation
        self.tf.transform = Transform(translation = pos, rotation = rot)
        self.tf.header.stamp = self.get_clock().now().to_msg()

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:

        # convert yaw to quaternion; ignore pitch and roll
        quat   = Quaternion()
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return(quat)

    def read_splines_info(self, trajectory: str) -> Tuple[PoseArray, float]:

        # read spline points from the CSV and store it in the main class
        rel_path = os.path.join(get_package_share_directory("canon"), "maps")
        raw_pts = csv.reader(open(os.path.expanduser(f"{rel_path}/{trajectory}.csv")), delimiter = ",")
        raw_pts = [[float(point[0]), float(point[1])] for point in raw_pts]

        # convert the trajectory to an array of poses
        points = PoseArray()
        points.header.frame_id = "map"
        for i in range(len(raw_pts)):
            pose = Pose()
            _next = (i + 1) % len(raw_pts)
            pose.position.x, pose.position.y = raw_pts[i][0], raw_pts[i][1]
            pose.orientation = self.yaw_to_quaternion(math.atan2(raw_pts[_next][1] - raw_pts[i][1], raw_pts[_next][0] - raw_pts[i][0]))
            points.poses.append(pose)
        
        # calculate the average spline resolution
        _res = []
        for i in range(len(points.poses)):
            _next = (i + 1) % len(points.poses)
            _ix = points.poses[i].position.x - points.poses[_next].position.y
            _iy = points.poses[i].position.y - points.poses[_next].position.y
            _res.append(math.hypot(_ix, _iy))
        _res = sum(_res) / len(_res)
        self.get_logger().info(f"extracted {len(points.poses)} {trajectory} points")
        return(points, _res)
    
    def localize_racecar_on_spline(self, state: Odometry, target_tup_idx: int, first_loc: bool) -> Tuple[int, float]:

        # decompose state vector
        x, y = state.pose.pose.position.x, state.pose.pose.position.y

        # if the spline localizer is not initialized, search the entire spline for the first localized index
        if not first_loc:
            _target_spline_ranges = []
            for i in range(len(self.splines_points[target_tup_idx].poses)):
                _xi, _yi = self.splines_points[target_tup_idx].poses[i].position.x, self.splines_points[target_tup_idx].poses[i].position.y
                _target_spline_ranges.append(math.hypot(x - _xi, y - _yi))
            _target_spline_lat_sep = min(_target_spline_ranges)
            _target_spline_loc_idx = _target_spline_ranges.index(_target_spline_lat_sep)
        
        # after initialization, check for points within the local area defined by self.dyn_range_loc
        else:
            _target_spline_ranges = []
            for i in range(-self.dyn_range_loc[target_tup_idx], self.dyn_range_loc[target_tup_idx]):
                _loc_idx = (self.known_loc_idx[target_tup_idx] + i) % len(self.splines_points[target_tup_idx].poses)
                _xi, _yi = self.splines_points[target_tup_idx].poses[_loc_idx].position.x, self.splines_points[target_tup_idx].poses[_loc_idx].position.y
                _target_spline_ranges.append(math.hypot(x - _xi, y - _yi))
            _target_spline_lat_sep = min(_target_spline_ranges)
            _target_spline_loc_idx = self.known_loc_idx[target_tup_idx] +_target_spline_ranges.index(_target_spline_lat_sep) 
            _target_spline_loc_idx -= self.dyn_range_loc[target_tup_idx]
            _target_spline_loc_idx %= len(self.splines_points[target_tup_idx].poses)
        
        # return the localized spline index and the lateral seperation to the target spline
        return(_target_spline_loc_idx, _target_spline_lat_sep)

    def localize_racecar(self, state: Odometry) -> Tuple[Tuple[int], Tuple[float]]:

        # localize the racecar for the active raceline and the track bounds
        _raceline_loc_idx, _raceline_lat_sep = self.localize_racecar_on_spline(state, PTConstants.OptimalRaceline, self.first_loc_found)
        _inner_bounds_loc_idx, _inner_bounds_lat_sep = self.localize_racecar_on_spline(state, PTConstants.TrackInnerBounds, self.first_loc_found)
        _outer_bounds_loc_idx, _outer_bounds_lat_sep = self.localize_racecar_on_spline(state, PTConstants.TrackOuterBounds, self.first_loc_found)

        # localizer initialization is always set true after the first cycle,
        # unless it is reset by an outside method
        if not self.first_loc_found:
            self.first_loc_found = True
        
        # return a tuple containing the localized index and lateral error
        return((_raceline_loc_idx, _inner_bounds_loc_idx, _outer_bounds_loc_idx), (_raceline_lat_sep, _inner_bounds_lat_sep, _outer_bounds_lat_sep))

    def assemble_reference_points(self, vel: float, target_tup_idx: int, reverse_pts: bool) -> PoseArray:

        # calcluate the skip index interval based on the current racecar velocity
        idx_skip = max(int(vel * self.time_tick / self.splines_res[target_tup_idx]), 1)
        loc_idx = self.known_loc_idx[target_tup_idx]
        reference = PoseArray()
        reference.header.frame_id = "map"

        # if reversed points are necessary (true, if track bounds are involved)
        # also add the current bounds point
        if reverse_pts:
            for i in range(self.horizon_len - 1):
                curr_idx = (loc_idx - (self.horizon_len - i - 1) * idx_skip) % len(self.splines_points[target_tup_idx].poses)
                reference.poses.append(self.splines_points[target_tup_idx].poses[curr_idx])
            reference.poses.append(self.splines_points[target_tup_idx].poses[loc_idx])

        # populate the evenly samped spline points
        for i in range(self.horizon_len):
            curr_idx = ((i + 1) * idx_skip + loc_idx) % len(self.splines_points[target_tup_idx].poses)
            reference.poses.append(self.splines_points[target_tup_idx].poses[curr_idx])
        return(reference)

    def assemble_references(self, state: Odometry) -> Tuple[PoseArray, PoseArray, PoseArray]:

        # calculate the current velocity and enforce bounds vioaltion
        vel = math.hypot(state.twist.twist.linear.x, state.twist.twist.linear.y)
        vel = min(max(vel, self.min_speed), self.max_speed)

        # assemble reference spline pose array for the active raceline and the track bounds
        _raceline_ref = self.assemble_reference_points(vel, PTConstants.OptimalRaceline, False)
        _inner_bounds_ref = self.assemble_reference_points(vel, PTConstants.TrackInnerBounds, True)
        _outer_bounds_ref = self.assemble_reference_points(vel, PTConstants.TrackOuterBounds, True)
        return(_raceline_ref, _inner_bounds_ref, _outer_bounds_ref)

    def yaw_from_quaternion(self, quat: Quaternion) -> float:

        # extract yaw from quaternion
        x = 1.0 - 2.0 * (math.pow(quat.y, 2) + math.pow(quat.z, 2))
        y = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        yaw = math.atan2(y, x)
        return(yaw)
    
    def iterate_current_cycle(self, x: float, y: float, yaw: float, vel: float, steer: float) -> Tuple[float, float, float]:

        # predict the immediate next cycle using a simple kinematic bicycle model of the racecar
        x += math.cos(yaw) * vel * self.time_tick
        y += math.sin(yaw) * vel * self.time_tick
        yaw += math.tan(steer) * vel / self.wheelbase_len * self.time_tick
        return(x, y, yaw)
    
    def predict_motion_for_candidate(self, state: Odometry, candidate: float) -> PoseArray:

        # decompose state vector
        x, y = state.pose.pose.position.x, state.pose.pose.position.y
        yaw = self.yaw_from_quaternion(state.pose.pose.orientation)
        vel = math.hypot(state.twist.twist.linear.x, state.twist.twist.linear.y)
        vel = min(max(vel, self.min_speed), self.max_speed)

        # predict the motion for the entire horizon
        prediction = PoseArray()
        prediction.header.frame_id = "map"
        for i in range(self.horizon_len):
            pose = Pose()
            x, y, yaw = self.iterate_current_cycle(x, y, yaw, vel, candidate)
            pose.position.x, pose.position.y = x, y
            pose.orientation = self.yaw_to_quaternion(yaw)
            prediction.poses.append(pose)
        return(prediction)
    
    def estimate_maneuver_cost(self, prediction: PoseArray, reference: PoseArray, _ref_lat_err: float) -> float:

        # add lateral seperation between each prediction and reference
        # use a inverse cubic cost decay model as pred distance increases
        # specifically target to prioritize smooth convergence based on lateral seperation to the raceline
        cost = 0.0
        for i in range(len(reference.poses)):
            cost_i = math.hypot(prediction.poses[i].position.x - reference.poses[i].position.x, prediction.poses[i].position.y - reference.poses[i].position.y)
            target_i = min(max(1, int(_ref_lat_err)), self.horizon_len)
            target_i = abs(target_i - i) + 1
            cost += cost_i / math.pow(target_i, 3)
        return(cost)
    
    def calculate_optimal_steer_candidate(self, state: Odometry, reference: PoseArray, _ref_lat_err: float) -> PoseArray:

        # gather predictions for each candidate in the steering candidates
        # compute the maneuver cost for each of the steering candidates
        # the minimum cost index corresponds to the optimal steering candidate
        predictions = [self.predict_motion_for_candidate(state, candidate) for candidate in self.candidates]
        candidates_costs = [self.estimate_maneuver_cost(prediction, reference, _ref_lat_err) for prediction in predictions]
        optimal_idx = candidates_costs.index(min(candidates_costs))

        # apply the output adjusted for steering damping ration
        steer = -1.0 * self.candidates[optimal_idx]
        print(steer)

        # enforce actuation rate limits for steering
        steer_p = self.command.target_wheel_angle
        steer_pi, steer_pd = steer_p + self.actuation_rate, steer_p - self.actuation_rate
        self.command.target_wheel_angle = steer_pi if steer > steer_pi else steer_pd if steer < steer_pd else steer

        # return optimal candidate steer prediction
        return(self.predict_motion_for_candidate(state, -self.command.target_wheel_angle))
    
    def calculate_drivetrain_inputs(self, state: Odometry) -> None:

        # get current velocity and tracking error
        vel = math.hypot(state.twist.twist.linear.x, state.twist.twist.linear.y)
        err = PTConstants.OptimalRacelineVelocity - vel

        if err > 0:
            power = 0.0
            power += err * LongitudinalGains.P_gain 
            power += (self.prev_error - err) * LongitudinalGains.D_gain
            power += sum(self.steady_err) / len(self.steady_err) * LongitudinalGains.I_gain

            power = min(max(0.0, power), self.power_max)
            brake = 0.0
            self.prev_error = err
            self.steady_err = self.steady_err[1:] + [err]

        elif abs(err) > self.coast_vel:
            power = 0.0
            brake = err / PTConstants.OptimalRacelineVelocity * self.brake_max
            brake = min(max(0.0, brake), self.brake_max)

        # enfore actuation rate limits for the powertrain
        power_p, brake_p = self.command.acceleration_pct, self.command.braking_pct
        power_pi, brake_pi = power_p + self.actuation_rate, brake_p + self.actuation_rate
        power_pd, brake_pd = power_p - self.actuation_rate, brake_p - self.actuation_rate
        self.command.acceleration_pct = power_pi if power > power_pi else power_pd if power < power_pd else power
        self.command.braking_pct = brake_pi if brake > brake_pi else brake_pd if brake < brake_pd else brake

    def path_tracker_node(self, state: Odometry) -> None:

        # execute path tracker node tasks sequentially
        self.assemble_dynamic_tf(state)
        self.known_loc_idx, lat_err = self.localize_racecar(state)
        refs = self.assemble_references(state)

        # computer steer and drivetrain inputs
        pt_ref = refs[PTConstants.OptimalRaceline]
        pt_lat_err = lat_err[PTConstants.OptimalRaceline]
        pred = self.calculate_optimal_steer_candidate(state, pt_ref, pt_lat_err)
        self.calculate_drivetrain_inputs(state)

        # pubish all the desired information
        self.tf_pub.sendTransform(self.tf)
        self.lat_err_pub.publish(Float64(data = lat_err[PTConstants.OptimalRaceline]))
        self.inner_bounds_pub.publish(refs[PTConstants.TrackInnerBounds])
        self.outer_bounds_pub.publish(refs[PTConstants.TrackOuterBounds])
        self.target_ref_pub.publish(refs[PTConstants.OptimalRaceline])
        self.optimal_pred_pub.publish(pred)
        self.command_pub.publish(self.command)

def path_tracker():
    rclpy.init()
    racecar_ns = str(sys.argv[1])
    node = PathTracker(racecar_ns)
    rclpy.spin(node)