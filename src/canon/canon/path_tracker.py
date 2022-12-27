#!/usr/bin/env python3

import rclpy
import math
import sys
import numpy as np

from dataclasses import dataclass
from typing import Tuple
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray, Quaternion, Point
from geometry_msgs.msg import Transform, TransformStamped, Vector3
from lgsvl_msgs.msg import VehicleControlData
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSReliabilityPolicy

@dataclass
class PTParams:

    # lateral control parameters
    max_steer: float = 2.0
    max_angle: float = 20.0
    gradient: float = 0.01
    output_scale: float = 2.5
    actuation_rate = 0.00125

    # racecar kinematic bicycle model params
    wheelbase_len: float = 3.0
    time_tick: float = 0.4
    horizon_len: int = 10
    to_min_index: int = 6
    cost_decay_f: float = 3.0
    pt_node_speed: float = 10.0
    abs_max_speed: float = 80.0

    # longitudinal control parameters
    P_gain: float = 0.05
    I_gain: float = 1e-6
    D_gain: float = 0.005
    history_len: int = 10
    coast_int: float = 5.0
    brake_damp_f = 0.001

# {x, y} index desc.
@dataclass
class PTIndex:
    x: int = 0
    y: int = 1

class PathTracker(Node):
    """
    Given a reference spline, attempt to minimize the lateral and longitudinal errors
    """

    def __init__(self, racecar_ns: str) -> None:

        super().__init__(f"{racecar_ns}_path_tracker")
        qos = QoSReliabilityPolicy.BEST_EFFORT

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
        self.curr_vel = 0.0
        self.prev_vel_err = 0.0
        self.steady_vel_err = np.zeros(PTParams.history_len)

        self.command = VehicleControlData()
        self.prediction = PoseArray()
        self.prediction.header.frame_id = "map"
        self.command_pub = self.create_publisher(VehicleControlData, f"/{racecar_ns}/command", qos)
        self.pred_pub = self.create_publisher(PoseArray, f"/{racecar_ns}/path_tracker/prediction", qos)

        self.create_subscription(PoseArray, f"/{racecar_ns}/path_server/spline", self.update_reference_spline, qos)
        self.create_subscription(Odometry, f"/{racecar_ns}/odometry", self.path_tracker_node, qos)
        self.create_subscription(Float64, f"/{racecar_ns}/path_server/ref_vel", self.longitudinal_control_node, qos)
    
    def update_reference_spline(self, spline: PoseArray) -> None:

        # everytime the reference changes, reset and populate the new spline points
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

    def iterate_current_cycle(self, x: float, y: float, yaw: float, steer: float) -> Tuple[float, float, float]:

        # predict the immediate next cycle using a simple kinematic bicycle model of the racecar
        _x = x + math.cos(yaw) * PTParams.pt_node_speed * PTParams.time_tick
        _y = y + math.sin(yaw) * PTParams.pt_node_speed * PTParams.time_tick
        _yaw = yaw + math.tan(steer) * PTParams.pt_node_speed / PTParams.wheelbase_len * PTParams.time_tick
        return(_x, _y, _yaw)
    
    def predict_motion_for_candidate(self, state: Odometry, steer: float) -> np.ndarray:

        # decompose state vector
        _x, _y = state.pose.pose.position.x, state.pose.pose.position.y
        _yaw = self.yaw_from_quaternion(state.pose.pose.orientation)

        # predict the motion for the entire horizon
        _prediction = np.empty([PTParams.horizon_len, 2])
        for i in range(PTParams.horizon_len):
            _x, _y, _yaw = self.iterate_current_cycle(_x, _y, _yaw, steer)
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
            _maneuver_cost += math.hypot(_eucl_x, _eucl_y) / (math.pow(abs(PTParams.to_min_index - i) + 1, PTParams.cost_decay_f))
        return(_maneuver_cost)

    def visualize_candidate_prediction(self, np_pred: np.ndarray) -> None:

        # convert the optimal prediction to a pose_array for rviz
        self.prediction.poses = []
        for i in range(PTParams.horizon_len - 1):
            _pose = Pose()
            _eucl_x = np_pred[i + 1][PTIndex.x] - np_pred[i][PTIndex.x]
            _eucl_y = np_pred[i + 1][PTIndex.y] - np_pred[i][PTIndex.y]
            _yaw = math.atan2(_eucl_y, _eucl_x)
            _pose.position = Point(x = np_pred[i][PTIndex.x], y = np_pred[i][PTIndex.y])
            _pose.orientation = Quaternion(z = math.sin(_yaw / 2.0), w = math.cos(_yaw / 2.0))
            self.prediction.poses.append(_pose)

    def longitudinal_control_node(self, ref_vel: Float64) -> None:

        # get the tracking error
        err = ref_vel.data - self.curr_vel

        # if the tracking error is positive, use full PID for power
        if err > 0:
            _power = err * PTParams.P_gain
            _power += (self.prev_vel_err - err) * PTParams.D_gain
            _power += sum(self.steady_vel_err) / len(self.steady_vel_err) * PTParams.I_gain
            self.prev_vel_err = err
            np.roll(self.steady_vel_err, 1)
            self.steady_vel_err[0] = err
            
            # enforce power bounds and actuation rate
            _power = min(max(0.0, _power), 1.0)
            _power_i = self.command.acceleration_pct + PTParams.actuation_rate
            _power_d = self.command.acceleration_pct - PTParams.actuation_rate
            _power_out = _power_i if _power > _power_i else _power_d if _power < _power_d else _power
            self.command.acceleration_pct = _power_out
            self.command.braking_pct = 0.0

        # allow for a "coasting" interval before brakes are engaged
        # ensure brake actuation rate
        elif abs(err) > PTParams.coast_int:
            self.command.acceleration_pct = 0.0
            _brake = ref_vel.data / PTParams.abs_max_speed * PTParams.brake_damp_f
            _brake = min(max(0.0, _brake), 1.0)
            _brake_i = self.command.braking_pct + PTParams.actuation_rate
            _brake_d = self.command.braking_pct - PTParams.actuation_rate
            _brake_out = _brake_i if _brake > _brake_i else _brake_d if _brake < _brake_d else _brake
            self.command.braking_pct = _brake_out

    def path_tracker_node(self, odom: Odometry) -> None:

        # keep a local copy of the current vehicle velocity state
        self.curr_vel = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)

        # assemble tf elements for the racecar
        _pos = Vector3(x = odom.pose.pose.position.x, y = odom.pose.pose.position.y)
        _rot = odom.pose.pose.orientation
        self.tf.transform = Transform(translation = _pos, rotation = _rot)
        self.tf.header.stamp = self.get_clock().now().to_msg()

        # sequentially compute the optimal steering
        # then, enfore the steering bounds and actuation rates
        _predictions = [self.predict_motion_for_candidate(odom, steer) for steer in self.candidates]
        _candidate_costs = [self.estimate_maneuver_cost(_prediction) for _prediction in _predictions]
        _optimal_idx = _candidate_costs.index(min(_candidate_costs))
        self.visualize_candidate_prediction(_predictions[_optimal_idx])
        _raw_steer = -self.candidates[_optimal_idx] * PTParams.output_scale
        _steer_i = self.command.target_wheel_angle + PTParams.actuation_rate
        _steer_d = self.command.target_wheel_angle - PTParams.actuation_rate
        _steer_out = _steer_i if _raw_steer > _steer_i else _steer_d if _raw_steer < _steer_d else _raw_steer
        self.command.target_wheel_angle = _steer_out

        # pubish all the desired information
        self.tf_pub.sendTransform(self.tf)
        self.pred_pub.publish(self.prediction)
        self.command_pub.publish(self.command)

def path_tracker():
    rclpy.init()
    racecar_ns = str(sys.argv[1])
    node = PathTracker(racecar_ns)
    rclpy.spin(node)

if __name__ == "__main__":
    path_tracker()