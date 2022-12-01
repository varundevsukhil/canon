#!/usr/bin/env python3

import rclpy
import math
import csv
import sys
import os
import atexit

from typing import List
from rclpy.node import Node
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from lgsvl_msgs.msg import VehicleControlData
from ament_index_python.packages import get_package_share_directory

POWER_AXIS = 1
STEER_AXIS = 2
POWER_LIMIT = 0.50
STEER_LIMIT = 0.125
RESOLUTION = 10.0

class BoundsLogger(Node):

    def __init__(self, racecar_ns: str, save_name: str) -> None:

        # init node variables
        super().__init__("bounds_logger")
        rel_path  = os.path.join(get_package_share_directory("canon"), "maps")
        self.save_file = csv.writer(open(os.path.expanduser("{}/{}.csv".format(rel_path, save_name)), mode = "w"), delimiter = ",", quoting = csv.QUOTE_NONNUMERIC)
        atexit.register(self.write_data_to_file)

        # the data we need is stored here
        self.points = []

        # publishers, subscribers and callbacks
        self.command_pub = self.create_publisher(VehicleControlData, f"/{racecar_ns}/command", 1)
        self.create_subscription(Joy, "/joy", self.read_teleop, 1)
        self.create_subscription(Odometry, f"/{racecar_ns}/odometry", self.bounds_logger_node, 1)
    
    def write_data_to_file(self) -> None:

        # save all points to the CSV file defined in arguments
        for point in self.points:
            self.save_file.writerow(point)
    
    def read_teleop(self, teleop: Joy) -> None:

        # assemble teleop command and publish to the racecar
        command = VehicleControlData()
        command.target_wheel_angle = teleop.axes[STEER_AXIS] * STEER_LIMIT * -1.0
        command.acceleration_pct = teleop.axes[POWER_AXIS] * POWER_LIMIT if teleop.axes[POWER_AXIS] > 0.0 else 0.0
        command.braking_pct = teleop.axes[POWER_AXIS] * POWER_LIMIT if teleop.axes[POWER_AXIS] <= 0.0 else 0.0
        self.command_pub.publish(command)
    
    def bounds_logger_node(self, state: Odometry) -> None:

        # get x and y points from the vehicle odometry
        x, y = round(state.pose.pose.position.x, 2), round(state.pose.pose.position.y, 2)

        # populate the first on start, this will become the reference
        if len(self.points) < 1:
            self.get_logger().info(f"recorded first point @ {x}, {y}")
            self.points.append([x, y])
        
        # append the x, y to the saved points every time the racecar moves the resolution distance
        elif math.hypot(x - self.points[-1][0], y - self.points[-1][1]) >= RESOLUTION:
            self.get_logger().info(f"recorded point @ {x}, {y}")
            self.points.append([x, y])

def bounds_logger() -> None:
    rclpy.init()
    racecar_ns = str(sys.argv[1])
    save_name = str(sys.argv[2])
    node = BoundsLogger(racecar_ns, save_name)
    rclpy.spin(node)