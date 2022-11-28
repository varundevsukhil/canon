#!/usr/bin/env python3

import math

from typing import List
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from nav_msgs.msg import Odometry

class LateralControl(object):
    
    def __init__(self) -> None:
        
        # candidate constructor options
        self.steer_max = 2.0
        self.gradient = 0.005
        self.candidates = self.create_gradiant_steer_candidates()
    
        # lateral control dynamics
        self.angle_max = 20.0
        self.damper_ratio = self.angle_max / self.steer_max
        
        # racecar geometric and controller parameters
        self.wheelbase = 3.0
        self.prediction_tick_sec = 0.4
        self.prediction_duration_N = 10
        self.prediction_vel_bound_low = 5.0
        self.prediction_vel_bound_high = 10.0
    
    def create_gradiant_steer_candidates(self) -> List[float]:
        
        # start with zero (neutral) steer and add RHS candidates
        # invert RHS polarity and index to create LHS candidates
        # convert all candidates to math.radians
        candidates = [0.0]
        while candidates[-1] < self.steer_max: candidates.append(candidates[-1] + self.gradient)
        negatives = [-candidate for candidate in candidates[1:]]
        candidates = [math.radians(candidate) for candidate in negatives[::-1] + candidates]
        return candidates
    
    def yaw_from_quat(self, quat: Quaternion) -> float:
        
        # extract yaw (radians) from quaternion rotation
        x   = 1.0 - 2.0 * (math.pow(quat.y, 2) + math.pow(quat.z, 2))
        y   = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        yaw = math.atan2(y, x)
        return(yaw)
    
    def quat_from_yaw(self, yaw: float) -> Quaternion:
        
        # convert vehicle heading from yaw(float, radians) to a quaternion rotation
        quat = Quaternion(x = 0.0, y = 0.0, z = math.sin(yaw / 2.0), w = math.cos(yaw / 2.0))
        return(quat)

    def predict_next_cycle(self, x: float, y: float, yaw: float, vel: float, steer: float) -> tuple(float, float, float):
        
        # use the linear kinematic bicycle model
        x += math.cos(yaw) * vel * self.prediction_tick_sec
        y += math.sin(yaw) * vel * self.prediction_tick_sec
        yaw += math.tan(steer) * vel / self.wheelbase * self.prediction_tick_sec
        return(x, y, yaw)
    
    def predict_full_horizon(self, curr_state: Odometry, candidate_steer: float) -> PoseArray:
        
        # decompose racecar state
        x, y = curr_state.pose.pose.position.x, curr_state.pose.pose.position.y
        yaw = self.yaw_from_quat(curr_state.pose.pose.orientation)
        vel = math.hypot(curr_state.twist.twist.linear.x, curr_state.twist.twist.linear.y)
        
        # enforce velocity bounds for prediction
        # at lower range, this will "hot start" the optimizer
        # at higher range, this will limit predictions going out of bounds
        vel = max(min(self.prediction_vel_bound_high, vel), self.prediction_vel_bound_low)
        
        # iteratively predict the trajectory of the racecar
        # while it travels at the current velocity and proposed steer
        predictions = PoseArray()
        for i in range(self.prediction_duration_N):
            prediction = Pose()
            x, y, yaw = self.predict_next_cycle(x, y, yaw, vel, candidate_steer)
            prediction.position.x, prediction.position.y = x, y
            prediction.orientation = self.quat_from_yaw(yaw)
            predictions.poses.append(prediction)
        return(predictions)
            