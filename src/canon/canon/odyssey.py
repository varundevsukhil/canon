#!/usr/bin/env python3

import rclpy
import math
import time

from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import UInt8, Float64, Bool, String
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
    gap_close_vel_offset: float = 2.0

    # trigger distance ranges for entry and exit
    attacker_in_range: float = 10.0
    overtake_complete: float = 15.0
    gap_follow_dist: float = 20.0

    # sequence timing parameters
    t_interval: float = 3.0

    # key codes for mode selection in console
    k_normal: str = "000"
    k_attack_range: str = "110"
    k_attack_wait: str = "210"
    k_attack_pass: str = "220"
    k_defense_range: str = "101"
    k_defense_wait: str = "301"
    k_defense_try: str = "302"
    k_defense_fail: str = "303"

@dataclass
class CarNS:

    # ROS qos reliability is best effort, ns anns id's
    car_1: int = 0
    car_2: int = 1

    # role definitions
    attacker: int = 0
    defender: int = 1

# coordinated attack dataclass sequence
@dataclass
class AttackSequence:
    waiting_for_trigger: int = 0
    passing_defender: int = 1
    passing_complete: int = 2

# the fornt strech for the oval is closer to the pagoda and the pitlane
@dataclass
class FrontStretch:

    # geometric centers for entry and exit
    exit_pt: Point = Point(x = -4.89, y = -277.08)
    entry_pt: Point = Point(x = -5.18, y = 650.59)

     # switch distance trigger thresholds
    exit_trig_dist: float = 25.0
    entry_trig_dist: float = 50.0

# the rear stretch consists of the pitlane exit and turn 3
@dataclass
class RearStretch:

    # geometric centers for entry and exit
    exit_pt: Point = Point(x = 728.24, y = 679.48)
    entry_pt: Point = Point(x = 729.21, y = -280.18)

     # switch distance trigger thresholds
    exit_trig_dist: float = 25.0
    entry_trig_dist: float = 50.0

class Racecar(object):
    """
    A holder class for each racecar's publishers and NS.
    """

    def __init__(self, node: Node, ns: int) -> None:
        
        # racecar ns and ROS qos
        self.ns = f"car_{ns + 1}"
        self.odom = Odometry()
        self.node = node
        qos = QoSReliabilityPolicy.BEST_EFFORT

        # internal spline and offset states
        self.cur_code = OCode.optimal
        self.cur_vel = OCode.defender_vel_offset

        # racecar's publishers
        self.code_pub = node.create_publisher(UInt8, f"/oracle/{self.ns}/code", qos)
        self.vel_offset_pub = node.create_publisher(Float64, f"/oracle/{self.ns}/vel_offset", qos)
        self.key_pub = node.create_publisher(String, f"/oracle/{self.ns}/key", qos)
    
    def publish_info(self, code: int, vel_offset: float, key: str) -> None:

        # get publisher data and send to LGSVL
        self.code_pub.publish(UInt8(data = code))
        self.vel_offset_pub.publish(Float64(data = vel_offset))
        self.key_pub.publish(String(data = key))

        if code != self.cur_code:
            self.cur_code = code
            self.node.get_logger().info(f"{self.ns} tracking spline code: {self.cur_code}")

        if vel_offset != self.cur_vel:
            self.cur_vel = vel_offset
            self.node.get_logger().info(f"{self.ns} requesting vel offset: {self.cur_vel}")
    
    def update_odometry(self, odom: Odometry) -> None:

        # save a copy of the racecar's odometry
        self.odom = odom
    
    def get_planar_pose(self) -> Point:

        # send only the {x, y} pose from the stored odometry info
        _point = Point(x = self.odom.pose.pose.position.x, y = self.odom.pose.pose.position.y)
        return(_point)

class Odyssey(Node):
    """
    All hail the odyssey node. This one is designed for deterministic overtakes and random defense.
    """

    def __init__(self) -> None:

        # ROS and related stuff
        super().__init__("odyssey")
        qos = QoSReliabilityPolicy.BEST_EFFORT

        # node variables
        self.car_1_attacker = [CarNS.car_1, CarNS.car_2]
        self.car_2_attacker = [CarNS.car_2, CarNS.car_1]
        self.role = self.car_2_attacker

        self.in_maneuver = False
        self.front_stretch = True
        self.attacker_states = [False, False, False]
        self.defender_blocking = False
        
        # node subscribers and spinners
        self.racecars = [Racecar(self, ns) for ns in [CarNS.car_1, CarNS.car_2]]
        self.gap_pub = self.create_publisher(Bool, "/oracle/attacker_closing_gap", qos)
        self.create_subscription(Odometry, "/car_1/odometry", self.racecars[CarNS.car_1].update_odometry, qos)
        self.create_subscription(Odometry, "/car_2/odometry", self.racecars[CarNS.car_2].update_odometry, qos)
        self.create_timer(0.05, self.odyssey_node)
    
    def odyssey_node(self) -> None:

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
                self.attacker_states = [False, False, False]
                self.defender_blocking = False
            
            # trigger an overtake
            if not self.attacker_states[AttackSequence.waiting_for_trigger]:
                self.attacker_states[AttackSequence.waiting_for_trigger] = True
                self.get_logger().info(f"car_{self.role[CarNS.attacker] + 1} is attempting an overtake")
                self.racecars[self.role[CarNS.defender]].publish_info(OCode.optimal, OCode.defender_vel_offset, OCode.k_defense_wait)
                self.racecars[self.role[CarNS.attacker]].publish_info(OCode.offset, OCode.defender_vel_offset, OCode.k_attack_wait)
                time.sleep(OCode.t_interval)
            
            # wait until the opponent is in range for an sequence update
            elif not self.defender_blocking:
                self.defender_blocking = True
                self.get_logger().info(f"car_{self.role[CarNS.defender] + 1} is attempting a position defense")
                self.racecars[self.role[CarNS.defender]].publish_info(OCode.offset, OCode.defender_vel_offset, OCode.k_defense_try)
                self.racecars[self.role[CarNS.attacker]].publish_info(OCode.offset, OCode.defender_vel_offset, OCode.k_attack_pass)
                time.sleep(OCode.t_interval)
            
            # pass the opponent and update the overtake sequence
            elif not self.attacker_states[AttackSequence.passing_defender]:
                _eucl_x = _attacker.x - _defender.x
                _eucl_y = _attacker.y - _defender.y
                self.racecars[self.role[CarNS.defender]].publish_info(OCode.offset, OCode.defender_vel_offset, OCode.k_defense_try)
                self.racecars[self.role[CarNS.attacker]].publish_info(OCode.optimal, OCode.attacker_vel_offset, OCode.k_attack_pass)
                if math.hypot(_eucl_x, _eucl_y) < OCode.attacker_in_range:
                    self.get_logger().info(f"car_{self.role[CarNS.attacker] + 1} in range for overtake")
                    self.attacker_states[AttackSequence.passing_defender] = True
            
            # complete the overtake seqence and fallback
            elif not self.attacker_states[AttackSequence.passing_complete]:
                _eucl_x = _attacker.x - _defender.x
                _eucl_y = _attacker.y - _defender.y
                if math.hypot(_eucl_x, _eucl_y) > OCode.overtake_complete:
                    self.get_logger().info(f"car_{self.role[CarNS.attacker] + 1} has passed the opponent")
                    self.racecars[self.role[CarNS.defender]].publish_info(OCode.optimal, OCode.defender_vel_offset, OCode.k_defense_fail)
                    self.racecars[self.role[CarNS.attacker]].publish_info(OCode.optimal, OCode.defender_vel_offset, OCode.k_defense_wait)
                    self.attacker_states[AttackSequence.passing_complete] = True
            
            # reverse the attacker and defender roles and the target stretch
            if not False in self.attacker_states:
                self.get_logger().info(f"car_{self.role[CarNS.attacker] + 1} overtake of car_{self.role[CarNS.defender] + 1} succesful")
                
                # reverse the attacker and defender roles for the racecars
                self.role = self.car_2_attacker if self.role == self.car_1_attacker else self.car_1_attacker
                self.get_logger().info(f"assigning car_{self.role[CarNS.attacker] + 1} as attacker and car_{self.role[CarNS.defender] + 1} as defender")

                # toggle between the front and rear target stretches
                self.front_stretch = not self.front_stretch
                self.get_logger().info("{} stretch is the next role reversal zone".format("front" if self.front_stretch else "rear"))

                # clear internal states
                self.in_maneuver = False
                self.attacker_states = [False, False, False]
                self.defender_blocking = False

        else:
            _eucl_x = _attacker.x - _stretch.entry_pt.x
            _eucl_y = _attacker.y - _stretch.entry_pt.y
            if math.hypot(_eucl_x, _eucl_y) < _stretch.entry_trig_dist:
                self.get_logger().info(f"car_{self.role[CarNS.attacker] + 1} in overtake zone")
                self.in_maneuver = True            

            # closing the gap to ensure faster maneuvers
            _eucl_a = _attacker.x - _defender.x
            _eucl_b = _attacker.y - _defender.y
            _closing_gap = math.hypot(_eucl_a, _eucl_b) > OCode.gap_follow_dist
            _gap_close_vel_offset = OCode.gap_close_vel_offset if _closing_gap else OCode.defender_vel_offset
            _attack_code = OCode.k_normal if _closing_gap else OCode.k_attack_range
            self.gap_pub.publish(Bool(data = _closing_gap))
            self.racecars[self.role[CarNS.defender]].publish_info(OCode.optimal, OCode.defender_vel_offset, OCode.k_defense_range)
            self.racecars[self.role[CarNS.attacker]].publish_info(OCode.optimal, _gap_close_vel_offset, _attack_code)

def odyssey():
    rclpy.init()
    node = Odyssey()
    rclpy.spin(node)

if __name__ == "__main__":
    odyssey()