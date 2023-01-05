#!/usr/bin/env python3

import sys
import os
import csv
import numpy as np
import matplotlib.pyplot as plt

from ament_index_python.packages import get_package_share_directory

# plot colors as defined in matplotlib.pyploy docs
CPLOT = ["k", "g", "r", "c", "m", "y", "b"]

class PlotFromData(object):
    """Plot all the curves in the target CSV."""

    def __init__(self, plot_data: str) -> None:

        # navigate to the location of the data file
        rel_path = os.path.join(get_package_share_directory("canon"), "plots")
        data_file = csv.reader(open(os.path.expanduser(f"{rel_path}/{plot_data}.csv")), delimiter = ",")

        # local variables
        labels = False
        points = []

        # first row is the label, the rest are data points
        for lpoint in data_file:
            if not labels:
                labels = lpoint
            else:
                points.append(lpoint)

        # convvert from a messy array to an np.array
        plot_points = np.zeros((len(labels), len(points)))
        for i in range(len(points)):
            for j in range(len(labels)):
                plot_points[j, i] = float(points[i][j])

        # create the plots
        _, fig = plt.subplots(1, 1)
        for k in range(1, len(labels)):
            fig.plot(plot_points[0], plot_points[k], f"-{CPLOT[k]}", label = labels[k])
        fig.set_aspect("equal", "box")
        plt.legend(loc = "lower right")
        plt.title("Lateral Tracking")
        plt.xlabel("Timestamp (s)")
        plt.ylabel("Lateral Error (m)")
        plt.show()

# start as main function
if __name__ == "__main__":
    data = str(sys.argv[1])
    plot = PlotFromData(data)