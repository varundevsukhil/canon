#!/usr/bin/env python3

import math
import os
import csv
import matplotlib.pyplot as plt
import numpy as np

from scipy import interpolate
from ament_index_python.packages import get_package_share_directory

T_WIDTH = 15.0

class BoundsCreator(object):

    def __init__(self) -> None:

        # csv read/write control
        rel_path = os.path.join(get_package_share_directory("canon"), "maps")
        inside_bounds_file = csv.writer(open(os.path.expanduser(f"{rel_path}/inside_bounds.csv"), mode = "w"), delimiter = ",", quoting = csv.QUOTE_NONNUMERIC)
        outside_bounds_file = csv.writer(open(os.path.expanduser(f"{rel_path}/outside_bounds.csv"), mode = "w"), delimiter = ",", quoting = csv.QUOTE_NONNUMERIC)
        centerline_file = csv.writer(open(os.path.expanduser(f"{rel_path}/centerline.csv"), mode = "w"), delimiter = ",", quoting = csv.QUOTE_NONNUMERIC)

        # read the raw logged points for the inner boundary
        inside_points = []
        for name in ["turn_1", "turn_2", "turn_3", "turn_4"]:
            data_file = csv.reader(open(os.path.expanduser(f"{rel_path}/{name}.csv")), delimiter = ",")
            for point in data_file:
                inside_points.append([float(point[0]), float(point[1])])
        
        # create the outside points usind a fixed track width
        outside_points = []
        for i in range(len(inside_points)):
            _next_idx = (i + 1) % len(inside_points)
            _theta = math.atan2(inside_points[_next_idx][1] - inside_points[i][1], inside_points[_next_idx][0] - inside_points[i][0])
            _theta -= math.radians(90.0) # rotate outide by 90 degrees
            _oxp = inside_points[i][0] + math.cos(_theta) * T_WIDTH
            _oyp = inside_points[i][1] + math.sin(_theta) * T_WIDTH
            outside_points.append([_oxp, _oyp])
        
        # interpolate the inside bounds
        # resample inside bounds until resolution is ~1.0m
        _ix = np.array([point[0] for point in inside_points])
        _iy = np.array([point[1] for point in inside_points])
        tck, _ = interpolate.splprep([_ix, _iy], s = 0, per = True)
        _ix, _iy = interpolate.splev(np.linspace(0, 1, int(len(_ix) * 25)), tck)

        # interpolate the inside bounds
        # resample outside bounds until resolution is ~1.0m
        _ox = np.array([point[0] for point in outside_points])
        _oy = np.array([point[1] for point in outside_points])
        tck, _ = interpolate.splprep([_ox, _oy], s = 0, per = True)
        _ox, _oy = interpolate.splev(np.linspace(0, 1, int(len(_ox) * 25)), tck)

        # find the center line
        center_points = []
        for i in range(len(_ix)):
            _ranges = []
            for j in range(len(_iy)):
                _ranges.append(math.hypot(_ix[i] - _ox[j], _iy[i] - _oy[j]))
            _min_range = _ranges.index(min(_ranges))
            _min_width = _ranges[_min_range] / 2.0
            _cix, _ciy = _ix[i], _iy[i]
            _cox, _coy = _ox[_min_range], _oy[_min_range]
            _theta = math.atan2(_coy - _ciy, _cox - _cix)
            _cx, _cy = _cix + math.cos(_theta) * _min_width, _ciy + math.sin(_theta) * _min_width
            center_points.append([_cx, _cy, _min_width])
        _cx = np.array([point[0] for point in center_points])
        _cy = np.array([point[1] for point in center_points])
        tck, _ = interpolate.splprep([_cx, _cy], s = 0, per = True)
        _cx, _cy = interpolate.splev(np.linspace(0, 1, int(len(_cx))), tck)
    
        # save splines to "maps" directory
        rel_path = os.path.join(get_package_share_directory("canon"), "maps")
        for i in range(len(_ix)):  inside_bounds_file.writerow([_ix[i], _iy[i]])
        for i in range(len(_ox)):  outside_bounds_file.writerow([_ox[i], _oy[i]])
        for i in range(len(_cx)):  centerline_file.writerow([_cx[i], _cy[i], T_WIDTH / 2.0, T_WIDTH / 2.0])

        # visualize the points
        _, fig = plt.subplots(1, 1)
        fig.plot(_ix, _iy, "r-")
        fig.plot(_ox, _oy, "g-")
        fig.plot(_cx, _cy, "b-")
        fig.set_aspect("equal", "box")
        plt.show()
      
if __name__ == "__main__":
    node = BoundsCreator()
