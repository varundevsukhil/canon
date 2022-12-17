#!/usr/bin/env python3

import math
import os
import csv
import matplotlib.pyplot as plt
import numpy as np

from scipy import interpolate
from ament_index_python.packages import get_package_share_directory

OFFSET = 5.0
OFFSET_VEL = 65.0

class BoundsCreator(object):

    def __init__(self) -> None:

        # csv read/write control
        rel_path = os.path.join(get_package_share_directory("canon"), "maps")
        original_spline_file = csv.reader(open(os.path.expanduser(f"{rel_path}/optimal.csv")), delimiter = ",")
        offset_spline_file = csv.writer(open(os.path.expanduser(f"{rel_path}/offset.csv"), mode = "w"), delimiter = ",", quoting = csv.QUOTE_NONNUMERIC)

        # read the raw logged points for the inner boundary
        _points = []
        for _point in original_spline_file:
            _points.append([float(_point[0]), float(_point[1])])
        _px = np.array([_point[0] for _point in _points])
        _py = np.array([_point[1] for _point in _points])
        _points = np.array(_points)

        # calculate the center point of the track geometry
        # then, add offsets for each point on the spline
        _cx = sum(_px) / len(_px)
        _cy = sum(_py) / len(_py)
        _offsets = []
        for i in range(len(_px)):
            _theta = math.atan2(_py[i] - _cy, _px[i] - _cx)
            _oix = _px[i] + OFFSET * math.cos(_theta)
            _oiy = _py[i]
            _offsets.append([_oix, _oiy])
        _ox = np.array([_offset[0] for _offset in _offsets])
        _oy = np.array([_offset[1] for _offset in _offsets])
            
        # # save splines to "maps" directory
        for i in range(len(_ox)):  offset_spline_file.writerow([_ox[i], _oy[i], OFFSET_VEL])

        # visualize the points
        _, fig = plt.subplots(1, 1)
        fig.plot(_px, _py, "r-")
        fig.plot(_ox, _oy, "g-")
        fig.set_aspect("equal", "box")
        plt.show()
      
if __name__ == "__main__":
    node = BoundsCreator()
