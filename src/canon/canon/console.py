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

# state definitions for the automatons
STATES = ["Vel. (m/s)", "Diff (m/s)", "Task [N]"]
ARGOS = [f"ARG {i}" for i in range(4)]
AUTOPASS = [f"AUP {i}" for i in range(4)]
KAVAL = [f"KVA {i}" for i in range(4)]

# ROS qos reliability is best effort, ns anns id's
@dataclass
class CarNS:
    car_1: int = 0
    car_2: int = 1

# console node parameters
@dataclass
class ConsoleParams:

    # number of states and associated values per racecar
    num_states: int = 3
    num_vals: int = 3

    # number of modes per automaton
    num_modes: int = 4
    num_autos: int = 3

    # Tk params for element relaive weight and padding from border
    weight: int = 1
    l_pad: int = 5

    # color code variations during mode transition
    inactive_ele: str = "black"
    inactive_bg: str = "white"
    active_ele: str = "red"
    active_bg: str = "green"

class Console(Node):
    """
    a Tk based console that displays critical racecar information.
    """

    def __init__(self, car_ns: str, opp_ns: str, loc: int) -> None:

        # ROS and related local node variables
        super().__init__("console")
        qos = QoSReliabilityPolicy.BEST_EFFORT
        self.vel = 0.0
        self.opp = 0.0
        self.keycode = "000"

        # Tk node config
        self.gui = Tk()
        self.gui.geometry(f"230x200+{loc}+110")
        self.gui.wm_attributes("-type", "splash")
        self.gui.wm_attributes("-topmost", True)
        l_name = "Car 1 [Blue]" if car_ns == "car_1" else "Car 2 [Orange]"

        # critical label frame with float variables
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

        # automaton and modes label frame
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
    
        # node spinners and subscribers
        self.create_subscription(Odometry, f"/{car_ns}/odometry", self.update_velocity, qos)
        self.create_subscription(Odometry, f"/{opp_ns}/odometry", self.monitor_opponent, qos)
        self.create_subscription(String, f"/oracle/{car_ns}/key", self.update_keycodes, qos)

    def update_velocity(self, odom: Odometry) -> None:

        # update ego racecar's velocity
        self.vel = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)
    
    def monitor_opponent(self, odom: Odometry) -> None:

        # monitor opponent racecar's velocity
        self.opp = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)

    def update_keycodes(self, key: String) -> None:

        # update mode and automaton keycode from the oracle/odyssey nodes
        self.keycode = key.data

    def update_mode(self, key: str, compare: str, automaton: int) -> str:

        # update mode for each automaton only when mode change event is recorded
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
        
        # if no change is recorded, simply return the previous key
        return(compare)

    def update_states(self) -> None:

        # update floating critical variables
        self.variables[STATES.index("Vel. (m/s)")].set(f"{round(self.vel, 2)} m/s")
        self.variables[STATES.index("Diff (m/s)")].set(f"{round(self.vel - self.opp, 2)} m/s")
        self.variables[STATES.index("Task [N]")].set(self.keycode)

        # update the mode/state for each automaton sequentially
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

if __name__ == "__main__":
    console()