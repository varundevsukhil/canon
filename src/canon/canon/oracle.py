#!/usr/bin/env python3

import rclpy
import math

from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import UInt8, Float64, Bool
from nav_msgs.msg import Odometry
from dataclasses import dataclass

# oracle and related params
@dataclass
class OCode:

    # trajectory enum codes
    pitlane: int = 0
    optimal: int = 1
    offset: int = 2

    # vel offsets for racecar agents
    attacker_vel_offset: float = 5.0
    defender_vel_offset: float = 0.0

    # trigger distance ranges for entry and exit
    attacker_in_range: float = 10.0
    overtake_complete: float = 25.0
    gap_follow_dist = 20.0

@dataclass
class CarNS:

    # ROS qos reliability is best effort, ns anns id's
    car_1: int = 0
    car_2: int = 1

    # role definitions
    attacker: int = 0
    defender: int = 1

# a simple dataclass of oracle sequence
@dataclass
class Sequence:
    passing_defender: int = 0
    passing_complete: int = 1

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
    exit_pt: Point = Point(x = 727.31, y = 513.55)
    entry_pt: Point = Point(x = 728.67, y = -114.24)

     # switch distance trigger thresholds
    exit_trig_dist: float = 25.0
    entry_trig_dist: float = 75.0

class Racecar(object):
    """
    A holder class for each racecar's publishers and NS.
    """

    def __init__(self, node: Node, ns: int) -> None:
        
        # racecar ns and ROS qos
        self.ns = f"car_{ns + 1}"
        self.odom = Odometry()
        qos = QoSReliabilityPolicy.BEST_EFFORT

        # racecar's publishers
        self.code_pub = node.create_publisher(UInt8, f"/oracle/{self.ns}/code", qos)
        self.vel_offset_pub = node.create_publisher(Float64, f"/oracle/{self.ns}/vel_offset", qos)
    
    def publish_info(self, code: int, vel_offset: float) -> None:

        # get publisher data and send to LGSVL
        self.code_pub.publish(UInt8(data = code))
        self.vel_offset_pub.publish(Float64(data = vel_offset))
    
    def update_odometry(self, odom: Odometry) -> None:

        # save a copy of the racecar's odometry
        self.odom = odom
    
    def get_planar_pose(self) -> Point:

        # send only the {x, y} pose from the stored odometry info
        _point = Point(x = self.odom.pose.pose.position.x, y = self.odom.pose.pose.position.y)
        return(_point)

class Oracle(Node):
    """
    All hail the oracle node. This one is designed for deterministic overtakes without defense.
    """

    def __init__(self) -> None:

        # ROS and related stuff
        super().__init__("oracle")
        qos = QoSReliabilityPolicy.BEST_EFFORT

        # node variables
        self.car_1_attacker = [CarNS.car_1, CarNS.car_2]
        self.car_2_attacker = [CarNS.car_2, CarNS.car_1]
        self.role = self.car_2_attacker

        self.in_maneuver = False
        self.front_stretch = True
        self.internal_states = [False, False]
        
        # node subscribers and spinners
        self.racecars = [Racecar(self, ns) for ns in [CarNS.car_1, CarNS.car_2]]
        self.gap_pub = self.create_publisher(Bool, "/oracle/attacker_closing_gap", qos)
        self.create_subscription(Odometry, "/car_1/odometry", self.racecars[CarNS.car_1].update_odometry, qos)
        self.create_subscription(Odometry, "/car_2/odometry", self.racecars[CarNS.car_2].update_odometry, qos)
        self.create_timer(0.05, self.oracle_node)
    
    def oracle_node(self) -> None:

        # get the poses and stretch information
        _attacker = self.racecars[CarNS.attacker].get_planar_pose()
        _defender = self.racecars[CarNS.defender].get_planar_pose()
        _stretch = FrontStretch if self.front_stretch else RearStretch

        # if, the racecar is already in a maneuver
        if self.in_maneuver:

            # montior the end of the current stretch, and declare a failure if it is triggered
            _eucl_x = _attacker.x - _stretch.exit_pt.x
            _eucl_y = _attacker.y - _stretch.exit_pt.y
            if math.hypot(_eucl_x, _eucl_y) < (_stretch.exit_trig_dist):
                self.get_logger().info(f"car_{self.role[CarNS.attacker] + 1} overtake failed")
                self.in_maneuver = False
                self.internal_states = [False, False]
            
            self.racecars[self.role[CarNS.attacker]].publish_info(OCode.offset, OCode.attacker_vel_offset)
            self.racecars[self.role[CarNS.defender]].publish_info(OCode.optimal, OCode.defender_vel_offset)

            # trigger an overtake
            if not self.internal_states[Sequence.passing_defender]:
                _eucl_x = _attacker.x - _defender.x
                _eucl_y = _attacker.y - _defender.y
                if math.hypot(_eucl_x, _eucl_y) < OCode.attacker_in_range:
                    self.get_logger().info(f"car_{self.role[CarNS.attacker] + 1} in range for overtake")
                    self.internal_states[Sequence.passing_defender] = True
            
            # pass the opponent and update the overtake sequence
            elif not self.internal_states[Sequence.passing_complete]:
                _eucl_x = _attacker.x - _defender.x
                _eucl_y = _attacker.y - _defender.y
                if math.hypot(_eucl_x, _eucl_y) > OCode.overtake_complete:
                    self.get_logger().info(f"car_{self.role[CarNS.attacker] + 1} has passed the opponent")
                    self.internal_states[Sequence.passing_complete] = True
            
            # reverse the attacker and defender roles and the target stretch
            if not False in self.internal_states:
                self.get_logger().info(f"car_{self.role[CarNS.attacker] + 1} overtake of car_{self.role[CarNS.defender] + 1} succesful")
                
                # reverse the attacker and defender roles for the racecars
                self.role = self.car_2_attacker if self.role == self.car_1_attacker else self.car_1_attacker
                self.get_logger().info(f"assigning car_{self.role[CarNS.attacker] + 1} as attacker and car_{self.role[CarNS.defender] + 1} as defender")

                # toggle between the front and rear target stretches
                self.front_stretch = not self.front_stretch
                self.get_logger().info("{} stretch is the next role reversal zone".format("front" if self.front_stretch else "rear"))

                # clear internal states
                self.in_maneuver = False
                self.internal_states = [False, False]

        else:
            _eucl_x = _attacker.x - _stretch.entry_pt.x
            _eucl_y = _attacker.y - _stretch.entry_pt.y
            if math.hypot(_eucl_x, _eucl_y) < _stretch.entry_trig_dist:
                self.get_logger().info(f"car_{self.role[CarNS.attacker] + 1} in overtake zone")
                self.in_maneuver = True            

            # closing the gap to ensure faster maneuvers
            _eucl_a = _attacker.x - _defender.x
            _eucl_b = _attacker.y - _defender.y
            closing_gap = math.hypot(_eucl_a, _eucl_b) > OCode.gap_follow_dist
            attacker_vel_offset = OCode.attacker_vel_offset if closing_gap else OCode.defender_vel_offset
            self.gap_pub.publish(Bool(data = closing_gap))
            self.racecars[self.role[CarNS.defender]].publish_info(OCode.optimal, OCode.defender_vel_offset)
            self.racecars[self.role[CarNS.attacker]].publish_info(OCode.optimal, attacker_vel_offset)

def oracle():
    rclpy.init()
    node = Oracle()
    rclpy.spin(node)

if __name__ == "__main__":
    oracle()