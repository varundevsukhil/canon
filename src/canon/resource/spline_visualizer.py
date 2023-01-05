#!/usr/bin/env python3

import os
import csv
import matplotlib.pyplot as plt

from ament_index_python.packages import get_package_share_directory

class SplineVisualizer(object):
    """A class to visualize all the static splines in this repository."""

    def __init__(self) -> None:

        # csv read/write control
        rel_path = os.path.join(get_package_share_directory("canon"), "maps")

        # read the interpolated points for the inside boundary
        _ix, _iy = [], []
        data_file = csv.reader(open(os.path.expanduser(f"{rel_path}/inside_bounds.csv")), delimiter = ",")
        for point in data_file: _ix.append(float(point[0])), _iy.append(float(point[1]))

        # read the interpolated points for the outside boundary
        _ox, _oy = [], []
        data_file = csv.reader(open(os.path.expanduser(f"{rel_path}/outside_bounds.csv")), delimiter = ",")
        for point in data_file: _ox.append(float(point[0])), _oy.append(float(point[1]))

        # read the interpolated points for the offset spline
        _cox, _coy = [], []
        data_file = csv.reader(open(os.path.expanduser(f"{rel_path}/offset.csv")), delimiter = ",")
        for point in data_file: _cox.append(float(point[0])), _coy.append(float(point[1]))

        # read the interpolated points for the optimal raceline
        _opx, _opy = [], []
        data_file = csv.reader(open(os.path.expanduser(f"{rel_path}/optimal.csv")), delimiter = ",")
        for point in data_file: _opx.append(float(point[0])), _opy.append(float(point[1]))

        # visualize the points
        _, fig = plt.subplots(1, 1)
        fig.plot(_ix, _iy, "k-")
        fig.plot(_ox, _oy, "k-")
        fig.plot(_cox, _coy, "r-")
        fig.plot(_opx, _opy, "g+")
        fig.set_aspect("equal", "box")
        plt.show()
      
if __name__ == "__main__":
    node = SplineVisualizer()
