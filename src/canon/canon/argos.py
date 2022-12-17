#!/usr/bin/env python3

import rclpy
import sys
import math

from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8, Float64
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
from dataclasses import dataclass
from rclpy.qos import QoSReliabilityPolicy

# trajectory enum codes
@dataclass
class SCCode:
    pitlane: int = 0
    optimal: int = 1
    offset_center: int = 2

@dataclass
class Pitlane:

    # geometric centers for entry and exit
    exit_pt: Point = Point(x = 710.25, y = -247.5)
    entry_pt: Point = Point(x = 13.5, y = 532.65)

    # switch distance trigger thresholds
    exit_trig_dist: float = 25.0
    entry_trig_dist: float = 75.0

# each static spline has a velocity offset associated with it
@dataclass
class VelocityOffset:
    pitlane: float = 0.0
    optimal: float = 0.0
    offset_center: float = 3.5

# TEST: use joycon to make spline switch
@dataclass
class JoyCode:
    pitlane: int = 1
    optimal: int = 2
    offset_center: int = 3

class ARGOS(Node):

    def __init__(self, racecar_ns: str) -> None:

        # ROS node requirements
        super().__init__(f"{racecar_ns}_argos")

        # we always spawn in the pitlane, so go to the racetrack ASAP
        self.on_racetrack = False
        self.to_pitlane = False
    
        # node publishers and data
        self.desired_code = SCCode.pitlane
        self.desired_offset = VelocityOffset.pitlane
        self.code_pub = self.create_publisher(UInt8, f"/{racecar_ns}/argos/spline_code", QoSReliabilityPolicy.BEST_EFFORT)
        self.vel_offset_pub = self.create_publisher(Float64, f"/{racecar_ns}/argos/vel_offset", QoSReliabilityPolicy.BEST_EFFORT)

        # node subscribers and spinners
        self.create_subscription(Odometry, f"/{racecar_ns}/odometry", self.spline_control_node, QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Joy, "/joy", self.update_spline_code, QoSReliabilityPolicy.BEST_EFFORT)
    
    def update_spline_code(self, joy: Joy) -> Node:

        # if the racecar is not on the race track, ignore switch commands
        if self.on_racetrack:

            # transition to optimal spline
            if joy.buttons[JoyCode.optimal] and self.desired_code != SCCode.optimal:
                self.get_logger().info("switching to optimal spline")
                self.desired_code = SCCode.optimal
                self.desired_offset = VelocityOffset.optimal
            
            # transition to optimal spline
            elif joy.buttons[JoyCode.offset_center] and self.desired_code != SCCode.offset_center:
                self.get_logger().info("switching to centreline offset spline")
                self.desired_code = SCCode.offset_center
                self.desired_offset = VelocityOffset.offset_center
            
            # request transition to pitlane
            elif joy.buttons[JoyCode.pitlane] and not self.to_pitlane:
                self.get_logger().info("requesting switch to pitlane spline")
                self.to_pitlane = True

    def spline_control_node(self, odom: Odometry) -> None:

        # exit the pitlane on to the race track before doing anything else
        if not self.on_racetrack:
            _eucl_x = odom.pose.pose.position.x - Pitlane.exit_pt.x
            _eucl_y = odom.pose.pose.position.y - Pitlane.exit_pt.y
            
            if math.hypot(_eucl_x, _eucl_y) < Pitlane.exit_trig_dist:
                self.get_logger().info("exiting the pitlane")
                self.desired_code = SCCode.optimal
                self.desired_offset = VelocityOffset.optimal
                self.on_racetrack = True
        
        # if pitlane entry is requested, wait until the transition is possible
        elif self.to_pitlane:
            _eucl_x = odom.pose.pose.position.x - Pitlane.entry_pt.x
            _eucl_y = odom.pose.pose.position.y - Pitlane.entry_pt.y
            
            if math.hypot(_eucl_x, _eucl_y) < Pitlane.entry_trig_dist:
                self.get_logger().info("entering the pitlane")
                self.desired_code = SCCode.pitlane
                self.desired_offset = VelocityOffset.pitlane
                self.on_racetrack = False
                self.to_pitlane = False
        
        # publish spline codes to the path_server node
        self.code_pub.publish(UInt8(data = self.desired_code))
        self.vel_offset_pub.publish(Float64(data = self.desired_offset))

def argos():
    rclpy.init()
    racecar_ns = str(sys.argv[1])
    node = ARGOS(racecar_ns)
    rclpy.spin(node)

if __name__ == "__main__":
    argos()