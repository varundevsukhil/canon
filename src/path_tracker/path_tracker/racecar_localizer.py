#!/usr/bin/env python3

import math

from geometry_msgs.msg import Transform, TransformStamped, Vector3
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

class RacecarLocalizer(object):
    
    def __init__(self, static_frame: str, moving_frame: str) -> None:
        
        # localizer variables
        self.trajectory_name = String()
        self.trajectory_points = Path()
        self.trajectory_resolution = 0.0
    
        # static and moving frame initiaization
        self.static_frame = static_frame
        self.moving_frame = moving_frame
    
    def assemble_transform(self, state: Odometry) -> TransformStamped:
        
        # assemble every element in a tf, except for the time stamp
        transform = TransformStamped()
        transform.header.frame_id = self.static_frame
        transform.child_frame_id = self.moving_frame
        translation = Vector3(x = state.pose.pose.position.x, y = state.pose.pose.position.y)
        rotation = state.pose.pose.orientation
        transform.transform = Transform(translation = translation, rotation = rotation)
        return(transform)
    
    def monitor_reference_trajectory(self, trajectory_name: String, trajectory_points: Path) -> None:
        
        # if the requested trajectory changes, update the trajectory points here
        if self.trajectory_name != trajectory_name:
            self.trajectory_name = trajectory_name
            self.trajectory_points = trajectory_points
    
    def localize_racecar(self, state: Odometry) -> tuple(int, float, TransformStamped):
        
        # decompose the odometry state vector, and assemble the transform
        x, y = state.pose.pose.position.x, state.pose.pose.position.y
        transform = self.assemble_transform(state)
        
        # find the closest Euclidean point on the reference trajectory to the racecar
        ranges = []
        for pose in self.trajectory_points.poses:
            ranges.append(math.hypot(x - pose.pose.position.x, y - pose.pose.position.y))
        loc_index = ranges.index(min(ranges))
        
        # compute lateral seperation using Heron's formulae
        _p1, _p2 = loc_index, (loc_index + 1) % len(self.trajectory_points.poses)
        _x1, _y1 = self.trajectory_points.poses[_p1].position.x, self.trajectory_points.poses[_p1].position.y
        _x2, _y2 = self.trajectory_points.poses[_p2].position_x, self.trajectory_points.poses[_p2].position.y
        ref_lateral_sep = abs((_x2 - _x1) * (_y1 - y) - (_x1 - x) * (_y2 - _y1)) / math.hypot(_x2 - _x1, _y2 - _y1)
        if (_x2 - _x1) * (y - _y1) - (_y2 - _y1) * (x - _x1) > 0.0: ref_lateral_sep *= -1.0
        return(loc_index, ref_lateral_sep, transform)