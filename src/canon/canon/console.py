#!/usr/bin/env python3

import rclpy
import time
import math
import sys

from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from typing import List
from tkinter import *
from tkinter.ttk import Progressbar
from dataclasses import dataclass

STATES = ["Vel. (m/s)", "Diff (m/s)", "Task [N]"]
ARGOS = [f"ARG {i}" for i in range(4)]
AUTOPASS = [f"AUP {i}" for i in range(4)]
KAVAL = [f"KVA {i}" for i in range(4)]

@dataclass
class CarNS:

    # ROS qos reliability is best effort, ns anns id's
    car_1: int = 0
    car_2: int = 1

@dataclass
class ConsoleParams:

    num_states: int = 3
    num_vals: int = 3

    num_modes: int = 4
    num_autos: int = 3

    weight: int = 1
    l_pad: int = 5

    inactive_ele: str = "black"
    inactive_bg: str = "white"

    active_ele: str = "red"
    active_bg: str = "green"

class Console(Node):

    def __init__(self, car_ns: str, opp_ns: str, loc: int) -> None:

        super().__init__("console")
        qos = QoSReliabilityPolicy.BEST_EFFORT
        self.vel = 0.0
        self.opp = 0.0
        self.keycode = "000"

        self.gui = Tk()
        self.gui.geometry(f"230x200+{loc}+75")
        self.gui.wm_attributes("-type", "splash")
        l_name = "Car 1 [Blue]" if car_ns == "car_1" else "Car 2 [Orange]"

        states = LabelFrame(self.gui, text = l_name, labelanchor = N)
        states.pack(fill = X, expand = YES)
        self.variables = [StringVar(), StringVar(), StringVar()]
        [states.grid_columnconfigure(i, weight = ConsoleParams.weight) for i in range (ConsoleParams.num_states)]
        [states.grid_rowconfigure(i, weight = ConsoleParams.weight) for i in range (ConsoleParams.num_vals)]

        [Label(states, text = state).grid(
            row = 0, 
            column = STATES.index(state), 
            padx = ConsoleParams.l_pad, 
            pady = ConsoleParams.l_pad, 
            sticky = EW) for state in STATES]
        [Label(states, relief = SUNKEN, textvariable = self.variables[i]).grid(
            row = 1, 
            column = i, 
            padx = ConsoleParams.l_pad, 
            pady = ConsoleParams.l_pad, 
            sticky = EW) for i in range (len(STATES))]

        self.modes = LabelFrame(self.gui, text = "States", labelanchor = N)
        self.modes.pack(fill = Y, expand = YES)
        [states.grid_columnconfigure(i, weight = 1) for i in range (ConsoleParams.num_modes)]
        [states.grid_rowconfigure(i, weight = 1) for i in range (ConsoleParams.num_autos)]

        self.argos_mode = ARGOS[3]
        self.autopass_mode = AUTOPASS[3]
        self.kaval_mode = KAVAL[3]

        for row in range(ConsoleParams.num_autos):
            t_mode = ARGOS if row == 0 else AUTOPASS if row == 1 else KAVAL
            [Label(
                self.modes, 
                relief = RAISED, 
                bg = ConsoleParams.inactive_bg, 
                fg = ConsoleParams.inactive_ele,
                text = t_mode[i]).grid(
                row = row, 
                column = i, 
                padx = ConsoleParams.l_pad, 
                pady = ConsoleParams.l_pad, 
                sticky = NSEW) for i in range(len(t_mode))]
    
        self.create_subscription(Odometry, f"/{car_ns}/odometry", self.update_velocity, qos)
        self.create_subscription(Odometry, f"/{opp_ns}/odometry", self.monitor_opponent, qos)
        self.create_subscription(String, f"/oracle/{car_ns}/key", self.update_keycodes, qos)

    def update_velocity(self, odom: Odometry) -> None:

        self.vel = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)
    
    def monitor_opponent(self, odom: Odometry) -> None:

        self.opp = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)

    def update_keycodes(self, key: String) -> None:

        self.keycode = key.data

    def update_mode(self, key: str, compare: str, automaton: int) -> str:

        if key != compare[-1]:
            t_mode = ARGOS if automaton == 0 else AUTOPASS if automaton == 1 else KAVAL
            [Label(
            self.modes, 
            relief = RAISED, 
            bg = ConsoleParams.inactive_bg if i != int(key) else ConsoleParams.active_bg, 
            fg = ConsoleParams.inactive_ele if i != int(key) else ConsoleParams.active_ele,
            text = t_mode[i]).grid(
            row = automaton, 
            column = i, 
            padx = ConsoleParams.l_pad, 
            pady = ConsoleParams.l_pad, 
            sticky = NSEW) for i in range(len(t_mode))]
            return(compare[:-1] + key)
        
        return(compare)

    def update_states(self) -> None:

        self.variables[STATES.index("Vel. (m/s)")].set(f"{round(self.vel, 2)} m/s")
        self.variables[STATES.index("Diff (m/s)")].set(f"{round(self.vel - self.opp, 2)} m/s")
        self.variables[STATES.index("Task [N]")].set(self.keycode)

        self.argos_mode = self.update_mode(self.keycode[0], self.argos_mode, 0)
        self.autopass_mode = self.update_mode(self.keycode[1], self.autopass_mode, 1)
        self.kaval_mode = self.update_mode(self.keycode[2], self.kaval_mode, 2)

def console():
    rclpy.init()
    car_ns = str(sys.argv[1])
    opp_ns = str(sys.argv[2])
    loc = int(sys.argv[3])
    node = Console(car_ns, opp_ns, loc)

    while rclpy.ok():
        node.update_states()
        node.gui.update()
        node.gui.update_idletasks()
        rclpy.spin_once(node)
        time.sleep(0.05)