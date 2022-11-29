#!/usr/bin/env python3

import csv
import math
import os

from geometry_msgs.msg import Quaternion, Pose, PoseArray
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Float64, Float64MultiArray
from ament_index_python.packages import get_package_share_directory

# directory control strings
PACKAGE_NAME = "path_tracker"
MAP_DIR_NAME = "maps"

# spline indices
X_POS_IDX = 0
Y_POS_IDX = 1
V_POS_IDX = 2

class PathServer(object):
    
    def __init__(self, default_trajectory: str) -> None:
        
        # trajectory data variables
        self.trajectory_name = String(data = default_trajectory)
        self.trajectory_points = Path()
        self.trajectory_velocity_profile = Float64MultiArray()
        self.trajectory_resolution = 0.0
        
        # racecar geometric and controller parameters
        self.wheelbase = 3.0
        self.prediction_tick_sec = 0.4
        self.prediction_duration_N = 10
        self.prediction_vel_bound_low = 5.0
        self.prediction_vel_bound_high = 10.0
        
        # construct default trajectory on start-up
        self.build_trajectory(self.trajectory_name)
    
    def build_trajectory(self, trajectory_name: String) -> None:
        
        # assemble path to trajectory resource and open using csv reader
        rel_path = os.path.join(get_package_share_directory(PACKAGE_NAME), MAP_DIR_NAME)
        file_data = csv.reader(open(os.path.expanduser(f"{rel_path}/{trajectory_name.data}")), delimiter = ",")
        
        # extract spline info from csv and populate trajectory info
        trajectory_points = Path()
        trajectory_velocity_profile = Float64MultiArray()
        trajectory_resolution = [0.0]
        for i in range(file_data):
            trajectory_point = Pose()
            _p1, _p2 = i, (i + 1) % len(file_data)
            _x1, _y1 = file_data[_p1][X_POS_IDX], file_data[_p1][Y_POS_IDX]
            _x2, _y2 = file_data[_p2][X_POS_IDX], file_data[_p2][Y_POS_IDX]
            trajectory_point.position.x, trajectory_point.position.y = _x1, _y1
            trajectory_point.orientation = self.quat_from_yaw(math.atan2(_y2 - _y1, _x2 - _x1))
            trajectory_points.poses.append(trajectory_point)
            trajectory_velocity_profile.data.append(file_data[_p1][V_POS_IDX])
            trajectory_resolution.append(math.hypot(_x1 - _x2, _y1 - _y2))
        
        # store trajectory info for path_server
        self.trajectory_name = trajectory_name
        self.trajectory_points = trajectory_points
        self.trajectory_velocity_profile = trajectory_velocity_profile
        self.trajectory_resolution = sum(trajectory_resolution) / len(trajectory_resolution)
    
    def get_path_server_data(self) -> tuple(String, Path, Float64MultiArray):
        
        # return internal path server variables
        return(self.trajectory_name, self.trajectory_points, self.trajectory_velocity_profile)
    
    def yaw_from_quat(self, quat: Quaternion) -> float:
        
        # extract yaw (radians) from quaternion rotation
        x = 1.0 - 2.0 * (math.pow(quat.y, 2) + math.pow(quat.z, 2))
        y = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        yaw = math.atan2(y, x)
        return(yaw)
    
    def quat_from_yaw(self, yaw: float) -> Quaternion:
        
        # convert vehicle heading from yaw(float, radians) to a quaternion rotation
        quat = Quaternion(x = 0.0, y = 0.0, z = math.sin(yaw / 2.0), w = math.cos(yaw / 2.0))
        return(quat)
    
    def create_path_tracker_reference(self, state: Odometry, loc_index: int) -> tuple(PoseArray, Float64MultiArray):
        
        # create new reference and velocity profile for every iteration
        reference = PoseArray()
        velocity_profile = Float64MultiArray()
        
        # enforce velocity bounds for reference creation
        # create a "skip" index that uniformly samples the raceline to create a reference pose array
        vel = math.hypot(state.twist.twist.linear.x, state.twist.twist.linear.y)
        vel = max(min(self.prediction_vel_bound_high, vel), self.prediction_vel_bound_low)
        skip_idx = max(1, int(vel * self.prediction_tick_sec / self.trajectory_resolution))
        
        # populate the reference and velocity profile equidistantly
        for i in range(self.prediction_duration_N):
            target_pose = Pose()
            int_idx = (i + 1) * skip_idx + loc_index
            target_pose.position.x, target_pose.position.y = self.trajectory_points.poses[int_idx].position.x, self.trajectory_points.poses[int_idx].position.y
            target_pose.orientation = self.trajectory_points.poses[int_idx].orientation
            reference.poses.append(target_pose)
            velocity_profile.data.append(Float64(data = self.trajectory_velocity_profile[int_idx].data))
        return(reference, velocity_profile)