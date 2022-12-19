#!/usr/bin/env python3

import rclpy
import math

from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import UInt8, Float64
from nav_msgs.msg import Odometry
from dataclasses import dataclass

# trajectory enum codes
@dataclass
class OCode:
    pitlane: int = 0
    optimal: int = 1
    offset_center: int = 2

@dataclass
class CarNS:

    # ROS qos reliability is best effort, ns anns id's
    car_1: int = 0
    car_2: int = 1

# the fornt strech for the oval is closer to the pagoda and the pitlane
@dataclass
class FrontStretch:

    # geometric centers for entry and exit
    exit_pt: Point = Point(x = -3.85, y = -125.14)
    entry_pt: Point = Point(x = -4.48, y = 524.64)

     # switch distance trigger thresholds
    exit_trig_dist: float = 25.0
    entry_trig_dist: float = 75.0

# the rear stretch consists of the pitlane exit and turn 3
@dataclass
class RearStretch:

    # geometric centers for entry and exit
    exit_pt: Point = Point(x = 728.67, y = -114.24)
    entry_pt: Point = Point(x = 727.31, y = 513.55)

     # switch distance trigger thresholds
    exit_trig_dist: float = 25.0
    entry_trig_dist: float = 75.0

# each static spline has a velocity offset associated with it
@dataclass
class VelOffset:
    pitlane: float = 0.0
    optimal: float = 0.0
    offset_center: float = 10.0

# successful overtake params
@dataclass
class SuccessfulOvertake:
    
    # vel offsets for racecar agents
    ego_vel_offset: float = 2.5
    opp_vel_offset: float = 0.0

    # trigger distance ranges for entry and exit
    ego_in_range_start: float = 25.0
    ego_out_of_range: float = 30.0

class Racecar(object):
    """
    A holder class for each racecar's publishers and NS.
    """

    def __init__(self, node: Node, ns: int) -> None:
        
        # racecar ns and ROS qos
        self.ns = f"car_{ns}"
        qos = QoSReliabilityPolicy.BEST_EFFORT

        # racecar's publishers
        self.code_pub = node.create_publisher(UInt8, f"/oracle/car_{ns}/code", qos)
        self.vel_offset_pub = node.create_publisher(Float64, f"/oracle/car_{ns}/vel_offset", qos)
    
    def publish_info(self, code: int, vel_offset: float) -> None:

        # get publisher data and send to LGSVL
        self.code_pub.publish(UInt8(data = code))
        self.vel_offset_pub.publish(Float64(data = vel_offset))

class Oracle(Node):
    """
    All hail the oracle node. This one is designed for deterministic overtakes without defense.
    """

    def __init__(self) -> None:

        # ROS and related stuff
        super().__init__("oracle")
        qos = QoSReliabilityPolicy.BEST_EFFORT
        self.car_ns = [1, 2]

        # node variables
        self.opp_odom = Odometry()
        self.in_maneuver = False
        self.maneuver_complete = False
        self.internal_states = [False, False]
        
        # node subscribers and spinners
        self.racecars = [Racecar(self, ns) for ns in self.car_ns]
        self.create_subscription(Odometry, "/car_1/odometry", self.update_opponent_odometry, qos)
        self.create_subscription(Odometry, "/car_2/odometry", self.oracle_node, qos)
    
    def update_opponent_odometry(self, odom: Odometry) -> None:

        # save a copy of the opponent's odometry
        self.opp_odom = odom
    
    def oracle_node(self, odom: Odometry) -> None:

        # if the maneuver has not started, wait for the trigger zone
        # request racecars follow the optimal raceline without velocity offsets
        if not self.in_maneuver:
            _eucl_x = odom.pose.pose.position.x - FrontStretch.entry_pt.x
            _eucl_y = odom.pose.pose.position.y - FrontStretch.entry_pt.y
            if math.hypot(_eucl_x, _eucl_y) < FrontStretch.entry_trig_dist:
                self.get_logger().info("starting overtake maneuver")
                self.in_maneuver = True
            self.racecars[CarNS.car_1].publish_info(OCode.optimal, VelOffset.optimal)
            self.racecars[CarNS.car_2].publish_info(OCode.optimal, VelOffset.optimal)
        
        # if the maneuver has started, wait for the maneuver to end
        elif self.in_maneuver:
            _eucl_x = odom.pose.pose.position.x - FrontStretch.exit_pt.x
            _eucl_y = odom.pose.pose.position.y - FrontStretch.exit_pt.y
            if math.hypot(_eucl_x, _eucl_y) < FrontStretch.exit_trig_dist:
                self.get_logger().info("overtake successful" if self.internal_states[1] else "overtake failed")
                self.in_maneuver = False
            
            # monitor the maneuver's completeness state
            if not self.maneuver_complete:
                if not False in self.internal_states:
                    self.maneuver_complete = True
                
                # request the ego to travel on the offset spline with a faster diff. velocity
                self.racecars[CarNS.car_1].publish_info(OCode.optimal, VelOffset.optimal)
                self.racecars[CarNS.car_2].publish_info(OCode.offset_center, VelOffset.offset_center)
                if not self.internal_states[0]:
                    _eucl_x = odom.pose.pose.position.x - self.opp_odom.pose.pose.position.x
                    _eucl_y = odom.pose.pose.position.y - self.opp_odom.pose.pose.position.y
                    if math.hypot(_eucl_x, _eucl_y) < SuccessfulOvertake.ego_in_range_start:
                        self.get_logger().info("ego in range for overtake")
                        self.internal_states[0] = True
                
                # wait until the ego has passed the opponent
                elif not self.internal_states[1]:
                    _eucl_x = odom.pose.pose.position.x - self.opp_odom.pose.pose.position.x
                    _eucl_y = odom.pose.pose.position.y - self.opp_odom.pose.pose.position.y
                    if math.hypot(_eucl_x, _eucl_y) > SuccessfulOvertake.ego_out_of_range:
                        self.get_logger().info("ego has passed the opponent")
                        self.internal_states[1] = True
            
            # by default, make both racecars follow the optimal spline without velocity offsets
            else:
                self.racecars[CarNS.car_1].publish_info(OCode.optimal, VelOffset.optimal)
                self.racecars[CarNS.car_2].publish_info(OCode.optimal, VelOffset.optimal)

def oracle():
    rclpy.init()
    node = Oracle()
    rclpy.spin(node)

if __name__ == "__main__":
    oracle()