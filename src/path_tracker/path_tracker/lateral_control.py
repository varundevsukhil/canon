#!/usr/bin/env python3
import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry


class LateralControl:
    def __init__(self) -> None:

        # optimizer global configs
        self.max_angle = 20.0
        self.max_steer = 2.0
        self.gradient = 0.005

        # create a list of candidate steering options
        self.candidates = [0.0]
        while self.candidates[-1] < self.max_steer:
            self.candidates.append(self.candidates[-1] + self.gradient)
        negatives = [-candidate for candidate in self.candidates[1:]]
        self.candidates = [
            math.radians(candidate) for candidate in negatives[::-1] + self.candidates
        ]

        # kinematic racecar model parameters
        self.wheelbase = 3.0
        self.horizon = 1.0
        self.time_tick = 0.04

    def yaw_from_quaternion(self, quat: Quaternion) -> float:
        x = 1.0 - 2.0 * (math.pow(quat.y, 2) + math.pow(quat.z, 2))
        y = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        return math.atan2(y, x)

    def quaternion_from_yaw(self, r: float, p: float, y: float) -> Quaternion:
        quat = Quaternion()
        quat.x = math.sin(r / 2) * math.cos(p / 2) * math.cos(y / 2) - math.cos(
            r / 2,
        ) * math.sin(p / 2) * math.sin(y / 2)
        quat.y = math.cos(r / 2) * math.sin(p / 2) * math.cos(y / 2) + math.sin(
            r / 2,
        ) * math.cos(p / 2) * math.sin(y / 2)
        quat.z = math.cos(r / 2) * math.cos(p / 2) * math.sin(y / 2) - math.sin(
            r / 2,
        ) * math.sin(p / 2) * math.cos(y / 2)
        quat.w = math.cos(r / 2) * math.cos(p / 2) * math.cos(y / 2) + math.sin(
            r / 2,
        ) * math.sin(p / 2) * math.sin(y / 2)
        return quat

    def iterate_current_cycle(
        self,
        x: float,
        y: float,
        yaw: float,
        vel: float,
        steer: float,
    ) -> tuple(float, float, float):

        # the linear kinematic bicycle model
        x += math.cos(yaw) * vel * self.time_tick
        y += math.sin(yaw) * vel * self.time_tick
        yaw += math.tan(steer) * vel / self.wheelbase * self.time_tick
        return (x, y, yaw)

    def predict_motion_for_steer(self, odom: Odometry, steer: float) -> PoseArray:

        # initial states for prediction
        x, y = odom.pose.pose.position.x, odom.pose.pose.position.y
        yaw = self.yaw_from_quaternion(odom.pose.pose.orientation)
        vel = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)

        predictions = PoseArray()
        for i in range(int(self.horizon / self.time_tick)):
            prediction = Pose()
            x, y, yaw = self.iterate_current_cycle(x, y, yaw, vel, steer)
            prediction.position.x, prediction.position.y = x, y
            prediction.orientation = self.quaternion_from_yaw(0.0, 0.0, yaw)
            predictions.poses.append(prediction)

    def calculate_maneuver_cost(
        self,
        predictions: PoseArray,
        reference: PoseArray,
        lat_sep: float,
    ) -> float:

        # cost proportionally increases with difference between prediction and reference
        # tighter predictions closer to the racecar are given more weight
        # lateral seperation produnes damped convergence to prevent spin-out on track
        cost = 0.0
        for iter in range(min(len(predictions.poses), len(reference.poses))):
            cost += math.hypot(
                predictions.poses[iter].position.x - reference.poses[iter].position.x,
                predictions.poses[iter].position.y - reference.poses[iter].position.y,
            )
            cost /= math.pow(iter + 1, 3)
            cost *= math.pow(lat_sep, 2)
        return cost

    def compute_optimal_steering(
        self,
        odom: Odometry,
        reference: PoseArray,
        lat_sep: float,
    ) -> float:

        # get motion predictions across all candidates
        candidate_predictions = []
        for candidate in self.candidates:
            candidate_predictions.append(self.predict_motion_for_steer(odom, candidate))

        # compute maneuver cost for all candidates
        candidate_cost = []
        for candidate_prediction in candidate_predictions:
            candidate_cost.append(
                self.calculate_maneuver_cost(candidate_prediction, reference, lat_sep),
            )

        # return the candidate steering that produces the minimum maneuver cost
        return (
            -1.0
            * self.candidates[candidate_cost.index(min(candidate_cost))]
            / max(self.candidates)
            / self.max_angle
            / self.max_steer
        )
