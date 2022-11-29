#!/usr/bin/env python3

import sys

from launch import LaunchDescription
from launch_ros.actions import Node

# launcher variable names
PATH_TRACKER_PACKAGE_NAME = "path_tracker"
PATH_TRACKER_EXEC = "path_tracker"
PATH_TRACKER_NODE = "{}_path_tracker"
PATH_TRACKER_MAIN_RACLINE = "IMS_Circuit_Optimal"
PATH_TRACKER_PASS_RACELINE = "IMS_Circuit_Overtake"
PATH_TRACKER_BLOCK_RACELINE = "IMS_Circuit_PosDefense"

class Racecar(object):
    
    def __init__(self, racecar_ns: str) -> None:
        
        # create the necessary nodes for each racecar
        self.path_tracker = Node(
            package = PATH_TRACKER_PACKAGE_NAME, 
            executable = PATH_TRACKER_EXEC, 
            name = PATH_TRACKER_NODE.format(racecar_ns), 
            arguments = [PATH_TRACKER_MAIN_RACLINE, PATH_TRACKER_PASS_RACELINE, PATH_TRACKER_BLOCK_RACELINE])

def generate_launch_description():
    
    # create a racecar class for each racecar entry from CLI
    racecars = [Racecar(str(sys.argv[ns])) for ns in range(1, len(sys.argv))]
    return(LaunchDescription([racecar.path_tracker for racecar in racecars]))