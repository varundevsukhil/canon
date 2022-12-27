#!/usr/bin/env python3

import os

from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

RVIZ_CONFIG = os.path.join(get_package_share_directory("canon"), "rviz", "argos.config.rviz")

class RacecarStack(object):
    """
    Create a list of nodes necessary for each racecar
    """

    def __init__(self, racecar_ns: str) -> None:

        self.racecar_ns = racecar_ns
        self.stack_nodes = ["path_server", "path_tracker", "argos"]
        self.nodes = [self.create_node("canon", node, [racecar_ns]) for node in self.stack_nodes]
    
    def create_node(self, package: str, executable: str, args: List[str]) -> Node:
        return(Node(
            package = package,
            executable = executable,
            name = f"{self.racecar_ns}_{executable}",
            arguments = args
        ))

class ARGOSStack(object):
    """
    Create a list of ARGOS and related nodes
    """

    def __init__(self) -> None:

        self.stack_nodes = ["odyssey"]
        self.nodes = [
            self.create_node("canon", "odyssey", ["oracle"]),
            self.create_node("canon", "visualize", ["car_1", "car_2"]),
            self.create_node("canon", "console", ["car_1", "car_2", "0"]),
            self.create_node("canon", "console", ["car_2", "car_1", "1920"]),
            self.create_node("rviz2", "rviz2", ["-d" + RVIZ_CONFIG])
        ]
    
    def create_node(self, package: str, executable: str, args: List[str]) -> Node:
        return(Node(
            package = package,
            executable = executable,
            name = f"{args[0]}_{executable}" if "car" in args[0] else executable,
            arguments = args
        ))

def generate_launch_description():
    argos = RacecarStack("car_1").nodes + RacecarStack("car_2").nodes + ARGOSStack().nodes
    return(LaunchDescription([node for node in argos]))