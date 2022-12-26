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

class ARGOSStack(object):
    """
    Create a list of ARGOS and related nodes
    """

    def __init__(self) -> None:

        self.stack_nodes = ["odyssey"]
        self.nodes = [
            self.create_node("canon", "odyssey"),
            self.create_console("canon", "car_1", "console", ["car_1", "car_2", "0"]),
            self.create_console("canon", "car_2", "console", ["car_2", "car_1", "1920"])]
    
    def create_node(self, package: str, executable: str) -> Node:
        return(Node(
            package = package,
            executable = executable,
            name = executable
        ))
    
    def create_console(self, package: str, ns: str, executable: str, args: List[str]) -> Node:
        return(Node(
            package = package,
            executable = executable,
            name = f"{ns}_{executable}",
            arguments = args
        ))

def generate_launch_description():
    racecar_1 = RacecarStack("car_1")
    racecar_2 = RacecarStack("car_2")
    argos = racecar_1.nodes + racecar_2.nodes + ARGOSStack().nodes
    return(LaunchDescription([node for node in argos]))