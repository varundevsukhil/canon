#!/usr/bin/env python3

import rclpy
import math
import sys
import numpy as np

from dataclasses import dataclass
from typing import Tuple
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from geometry_msgs.msg import Transform, TransformStamped, Vector3
from lgsvl_msgs.msg import VehicleControlData
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSReliabilityPolicy

@dataclass
class PTParams:

    max_steer: float = 2.0
    max_angle: float = 20.0
    gradient: float = 0.005

    wheelbase_len: float = 3.0
    time_tick: float = 0.4
    horizon_len: int = 10
    min_speed: float = 20.0
    max_speed: float = 40.0

@dataclass
class PTIndex:
    x: int = 0
    y: int = 1

class PathTracker(Node):

    def __init__(self, racecar_ns: str) -> None:

        super().__init__(f"{racecar_ns}_path_tracker")

        # dynamic transform
        self.tf_pub = TransformBroadcaster(self)
        self.tf = TransformStamped()
        self.tf.child_frame_id = racecar_ns
        self.tf.header.frame_id = "map"

        _gradient_range = int(PTParams.max_steer / PTParams.gradient)
        _candidates = [_candidate * PTParams.gradient for _candidate in range(_gradient_range)]
        _negatives = [-_candidate for _candidate in _candidates[1:]]
        _candidates = _negatives[::-1] + _candidates
        self.candidates = [math.radians(_candidate) for _candidate in _candidates]

        self.reference = np.empty([PTParams.horizon_len, 2])
        self.lat_err = 0.0

        self.command = VehicleControlData()
        self.prediction = PoseArray()
        self.prediction.header.frame_id = "map"
        self.command_pub = self.create_publisher(VehicleControlData, f"/{racecar_ns}/command", QoSReliabilityPolicy.BEST_EFFORT)
        self.pred_pub = self.create_publisher(PoseArray, f"/{racecar_ns}/path_tracker/prediction", QoSReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(PoseArray, f"/{racecar_ns}/path_server/spline", self.update_reference_spline, QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, f"/{racecar_ns}/odometry", self.path_tracker_node, QoSReliabilityPolicy.BEST_EFFORT)
    
    def update_reference_spline(self, spline: PoseArray) -> None:

        _reference = np.empty([PTParams.horizon_len, 2])
        for i in range(len(spline.poses)):
            _pose = spline.poses[i]
            _reference[i] = np.array([_pose.position.x, _pose.position.y])
        self.reference = _reference
    
    def yaw_from_quaternion(self, quat: Quaternion) -> float:

        # extract yaw from quaternion
        _x = 1.0 - 2.0 * (math.pow(quat.y, 2) + math.pow(quat.z, 2))
        _y = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        return(math.atan2(_y, _x))

    def iterate_current_cycle(self, x: float, y: float, yaw: float, vel: float, steer: float) -> Tuple[float, float, float]:

        # predict the immediate next cycle using a simple kinematic bicycle model of the racecar
        _x = x + math.cos(yaw) * vel * PTParams.time_tick
        _y = y + math.sin(yaw) * vel * PTParams.time_tick
        _yaw = yaw + math.tan(steer) * vel / PTParams.wheelbase_len * PTParams.time_tick
        return(_x, _y, _yaw)
    
    def predict_motion_for_candidate(self, state: Odometry, steer: float) -> np.ndarray:

        # decompose state vector
        _x, _y = state.pose.pose.position.x, state.pose.pose.position.y
        _yaw = self.yaw_from_quaternion(state.pose.pose.orientation)
        _vel = math.hypot(state.twist.twist.linear.x, state.twist.twist.linear.y)
        _vel = min(max(_vel, PTParams.min_speed), PTParams.max_speed)

        # predict the motion for the entire horizon
        _prediction = np.empty([PTParams.horizon_len, 2])
        for i in range(PTParams.horizon_len):
            _x, _y, _yaw = self.iterate_current_cycle(_x, _y, _yaw, _vel, steer)
            _prediction[i] = np.array([_x, _y])
        return(_prediction)
    
    def estimate_maneuver_cost(self, prediction: np.ndarray) -> float:

        # add lateral seperation between each prediction and reference
        # use a inverse cubic cost decay model as pred distance increases
        # specifically target to prioritize smooth convergence based on lateral seperation to the raceline
        _maneuver_cost = 0.0
        for i in range(PTParams.horizon_len):
            _eucl_x = prediction[i][PTIndex.x] - self.reference[i][PTIndex.x]
            _eucl_y = prediction[i][PTIndex.y] - self.reference[i][PTIndex.y]
            _maneuver_cost += math.hypot(_eucl_x, _eucl_y) % (math.pow(i + 1, 3))
        return(_maneuver_cost)

    def visualize_candidate_prediction(self, np_pred: np.ndarray) -> None:

        self.prediction.poses = []
        for i in range(PTParams.horizon_len - 1):
            _pose = Pose()
            _eucl_x = np_pred[i + 1][PTIndex.x] - np_pred[i][PTIndex.x]
            _eucl_y = np_pred[i + 1][PTIndex.y] - np_pred[i][PTIndex.y]
            _yaw = -math.atan2(_eucl_y, _eucl_x)
            _pose.position = Point(x = np_pred[i][PTIndex.x], y = np_pred[i][PTIndex.y])
            _pose.orientation = Quaternion(z = math.sin(_yaw / 2.0), w = math.cos(_yaw / 2.0))
            self.prediction.poses.append(_pose)

    def path_tracker_node(self, odom: Odometry) -> None:

        # assemble tf elements for the racecar
        _pos = Vector3(x = odom.pose.pose.position.x, y = odom.pose.pose.position.y)
        _rot = odom.pose.pose.orientation
        self.tf.transform = Transform(translation = _pos, rotation = _rot)
        self.tf.header.stamp = self.get_clock().now().to_msg()

        _predictions = [self.predict_motion_for_candidate(odom, steer) for steer in self.candidates]
        _candidate_costs = [self.estimate_maneuver_cost(_prediction) for _prediction in _predictions]
        _optimal_idx = _candidate_costs.index(min(_candidate_costs))
        _optimal_steer = -self.candidates[_optimal_idx]
        
        self.visualize_candidate_prediction(_predictions[_optimal_idx])

        # pubish all the desired information
        self.tf_pub.sendTransform(self.tf)
        self.pred_pub.publish(self.prediction)

def path_tracker():
    rclpy.init()
    racecar_ns = str(sys.argv[1])
    node = PathTracker(racecar_ns)
    rclpy.spin(node)

if __name__ == "__main__":
    path_tracker()