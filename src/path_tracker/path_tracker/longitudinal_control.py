#!/usr/bin/env python3

import math

from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

class LongitudinalControl(object):
    
    def __init__(self) -> None:
        
        # racecar longitudinal bounds
        self.speed_min = 0.0
        self.speed_max = 45.0
        self.power_min = 0.0
        self.power_max = 1.0
        self.brake_min = 0.0
        self.brake_max = 1.0
        self.coast_interval = 5.0
        
        # controller parameters
        self.P_gain = 0.15
        self.I_gain = 0.15
        self.D_gain = 0.02
        self.prev_error = 0.0
        self.steady_error = [0.0] * 100
        
        # commanded power and brake inputs
        self.command_power = 0.0
        self.command_brake = 0.0
    
    def compute_powertrain_inputs(self, state: Odometry, profile: Float64MultiArray) -> tuple(float, float):
        
        # the target speed is the average of the velocity profile
        # but it's never in violation of the velocity bounds
        # the difference between the setpoint (target) and the current speed is the tracking error
        target_vel = sum(profile.data) / len(profile.data)
        target_vel = min(max(target_vel, self.speed_min), self.speed_max)
        curr_vel = math.hypot(state.twist.twist.linear.x, state.twist.twist.linear.y)
        tracking_error = target_vel - curr_vel
        
        # a positive tracking error requires a full PID control loop for power with zero brakes
        # and a negative tracking error below the coast interval requires a proportional braking effort
        # a tuple containing the (power, brake) inputs are returned at the end
        if tracking_error > 0.0:
            self.command_power = self.power_min
            self.command_brake = self.power_max
            self.command_power += tracking_error * self.P_gain
            self.command_power += (self.prev_error - tracking_error) * self.D_gain
            self.command_power += sum(self.steady_error) / len(self.steady_error) * self.I_gain
            self.command_power *= 1.0 + target_vel / self.speed_max
            self.command_power = min(max(self.command_power, self.power_min), self.power_max)
            self.prev_error = tracking_error
            self.steady_error = self.steady_error[1:] + [tracking_error]
        elif abs(tracking_error) > self.coast_interval:
            self.command_power = self.power_min
            self.command_brake = abs(tracking_error) / self.speed_max * self.brake_max
            self.command_brake = min(max(self.command_brake, self.brake_min), self.brake_max)
        return(self.command_power, self.command_brake)