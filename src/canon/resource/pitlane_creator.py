#!/usr/bin/env python3

import math
import os
import csv
import matplotlib.pyplot as plt
import numpy as np

from ament_index_python.packages import get_package_share_directory

PL_WIDTH = 15.0
PL_VEL = 15.0
DESIRED_RES = 2.0

class PitlaneCreator(object):

    def __init__(self) -> None:

        # csv read/write control
        rel_path = os.path.join(get_package_share_directory("canon"), "maps")
        pitlane_file = csv.writer(open(os.path.expanduser(f"{rel_path}/pitlane_centerline.csv"), mode = "w"), delimiter = ",", quoting = csv.QUOTE_NONNUMERIC)

        # read the raw logged points for the pitlane boundary
        pitlane_outside_bounds = []
        for name in ["pit_entry", "pit_exit"]:
            data_file = csv.reader(open(os.path.expanduser(f"{rel_path}/{name}.csv")), delimiter = ",")
            for point in data_file:
                pitlane_outside_bounds.append([float(point[0]), float(point[1])])
        
        # extend the pitlane exit region
        for i in range(25):
            [_ix, _iy] = pitlane_outside_bounds[-1]
            [_ipx, _ipy] = pitlane_outside_bounds[-2]
            _theta = math.atan2(_iy - _ipy, _ix - _ipx)
            _nx = _ix + 10.0 * math.cos(_theta)
            _ny = _iy + 10.0 * math.sin(_theta)
            pitlane_outside_bounds.append([_nx, _ny])

        _pox = np.array([point[0] for point in pitlane_outside_bounds])
        _poy = np.array([point[1] for point in pitlane_outside_bounds])
        
        # create the inside pitlane points usind a fixed track width
        pitlane_inside_bounds = []
        for i in range(len(pitlane_outside_bounds) - 1):
            _next_idx = (i + 1) % len(pitlane_outside_bounds)
            _theta = math.atan2(pitlane_outside_bounds[_next_idx][1] - pitlane_outside_bounds[i][1], pitlane_outside_bounds[_next_idx][0] - pitlane_outside_bounds[i][0])
            _theta += math.radians(90.0) # rotate inside by 90 degrees
            _oxp = pitlane_outside_bounds[i][0] + math.cos(_theta) * PL_WIDTH
            _oyp = pitlane_outside_bounds[i][1] + math.sin(_theta) * PL_WIDTH
            pitlane_inside_bounds.append([_oxp, _oyp])
        _pix = np.array([point[0] for point in pitlane_inside_bounds])
        _piy = np.array([point[1] for point in pitlane_inside_bounds])
        
        # find the pitlane centerline
        center_points = []
        for i in range(len(_pix)):
            _ranges = []
            for j in range(len(_piy)):
                _ranges.append(math.hypot(_pix[i] - _pox[j], _piy[i] - _poy[j]))
            _min_range = _ranges.index(min(_ranges))
            _min_width = _ranges[_min_range] / 2.0
            _cix, _ciy = _pix[i], _piy[i]
            _cox, _coy = _pox[_min_range], _poy[_min_range]
            _theta = math.atan2(_coy - _ciy, _cox - _cix)
            _cx, _cy = _cix + math.cos(_theta) * _min_width, _ciy + math.sin(_theta) * _min_width
            center_points.append([_cx, _cy, _min_width])
        _pcx = np.array([point[0] for point in center_points])
        _pcy = np.array([point[1] for point in center_points])

        # interpolate the pitlane centerline manually
        interpolated_centerline = [center_points[0]]
        for i in range(1, len(center_points)):
            while math.hypot(interpolated_centerline[-1][0] - center_points[i][0], interpolated_centerline[-1][1] - center_points[i][1]) > DESIRED_RES:
                _theta = math.atan2(center_points[i][1] - interpolated_centerline[-1][1], center_points[i][0] - interpolated_centerline[-1][0])
                _ipx = interpolated_centerline[-1][0] + math.cos(_theta) * DESIRED_RES
                _ipy = interpolated_centerline[-1][1] + math.sin(_theta) * DESIRED_RES
                interpolated_centerline.append([_ipx, _ipy])
        _ipcx = np.array([point[0] for point in interpolated_centerline])
        _ipcy = np.array([point[1] for point in interpolated_centerline])

        # save the pitlane centerline to the "maps" directory
        for i in range(len(_ipcx)):
            pitlane_file.writerow([_ipcx[i], _ipcy[i], PL_VEL])
        
        # visualize the points
        _, fig = plt.subplots(1, 1)
        fig.plot(_pix, _piy, "k+")
        fig.plot(_pox, _poy, "k+")
        fig.plot(_pcx, _pcy, "r+")
        fig.plot(_ipcx, _ipcy, "g+")
        fig.set_aspect("equal", "box")
        plt.show()

if __name__ == "__main__":
    node = PitlaneCreator()