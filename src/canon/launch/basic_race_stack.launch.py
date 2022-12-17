#!/usr/bin/env python3

from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node

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

def generate_launch_description():
    racecar_1 = RacecarStack("car_1")
    racecar_2 = RacecarStack("car_2")
    return(LaunchDescription([node for node in racecar_1.nodes]))