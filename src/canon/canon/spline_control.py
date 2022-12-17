#!/usr/bin/env python3

import rclpy
import sys
import math

from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8
from geometry_msgs.msg import Point
from dataclasses import dataclass
from rclpy.qos import QoSReliabilityPolicy

# trajectory enum codes
@dataclass
class SCCode:
    pitlane: int = 0
    optimal: int = 1
    overtake_dynamic: int = 2
    defense_dynamic: int = 3

@dataclass
class Pitlane:

    # geometric centers for entry and exit
    exit_pt: Point = Point(x = 710.25, y = -247.5)
    entry_pt: Point = Point(x = 13.5, y = 532.65)

    # switch distance trigger thresholds
    exit_trig_dist: float = 75.0
    entry_trig_dist: float = 150.0

class SplineControl(Node):

    def __init__(self, racecar_ns: str) -> None:

        # ROS node requirements
        super().__init__(f"{racecar_ns}_spline_control")

        # we always spawn in the pitlane, so go to the racetrack ASAP
        self.on_racetrack = False
    
        # node publishers and data
        self.desired_code = SCCode.pitlane
        self.code_pub = self.create_publisher(UInt8, f"/{racecar_ns}/argos/spline_code", QoSReliabilityPolicy.BEST_EFFORT)

        # node subscribers and spinners
        self.create_subscription(Odometry, f"/{racecar_ns}/odometry", self.spline_control_node, QoSReliabilityPolicy.BEST_EFFORT)
    
    def spline_control_node(self, odom: Odometry) -> None:

        # exit the pitlane on to the race track before doing anything else
        if not self.on_racetrack:
            _eucl_x = odom.pose.pose.position.x - Pitlane.exit_pt.x
            _eucl_y = odom.pose.pose.position.y - Pitlane.exit_pt.y
            
            if math.hypot(_eucl_x, _eucl_y) < Pitlane.exit_trig_dist:
                self.desired_code = SCCode.optimal
                self.on_racetrack = True
        
        # publish spline codes to the path_server node
        self.code_pub.publish(UInt8(data = self.desired_code))

def spline_control():
    rclpy.init()
    racecar_ns = str(sys.argv[1])
    node = SplineControl(racecar_ns)
    rclpy.spin(node)

if __name__ == "__main__":
    spline_control()