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
        
        # create an offset for the time-stamp:
        offset = plot_points[0, 0]
        for i in range(len(points)):
            plot_points[0, i] -= offset

        # create the plots and layout
        layout = plt.figure(constrained_layout=True).subplot_mosaic(
            """
            AAAAA
            BCDEF
            """
        )

        # set the combined plots
        layout["A"].plot(plot_points[0], plot_points[1], f"-{CPLOT[1]}")
        layout["A"].plot(plot_points[0], plot_points[2], f"-{CPLOT[2]}")
        layout["A"].plot(plot_points[0], plot_points[3], f"-{CPLOT[3]}")
        layout["A"].plot(plot_points[0], plot_points[4], f"-{CPLOT[4]}")
        layout["A"].plot(plot_points[0], plot_points[5], f"-{CPLOT[5]}")
        layout["A"].get_xaxis().set_visible(False)
        layout["A"].set_ylabel("Lateral separation to the raceline (m/s)")
        layout["A"].set_title("Comined lateral tracking performance")

        # set the plots for path tracker
        layout["B"].plot(plot_points[1], plot_points[0], f"-{CPLOT[1]}")
        layout["B"].get_yaxis().set_visible(False)
        layout["B"].set_xlabel("path-tracker\nlateral error\n(m/s)")
        layout["B"].set_xlim([0.0, 17.5])

        # set the plots for adaptive look-ahead pure-pursuit
        layout["C"].plot(plot_points[2], plot_points[0], f"-{CPLOT[2]}")
        layout["C"].get_yaxis().set_visible(False)
        layout["C"].set_xlabel("pure-pursuit\nlateral error\n(m/s)")
        layout["C"].set_xlim([0.0, 17.5])

        # set the plots for Stanley control
        layout["D"].plot(plot_points[3], plot_points[0], f"-{CPLOT[3]}")
        layout["D"].get_yaxis().set_visible(False)
        layout["D"].set_xlabel("Stanley-control\nlateral error\n(m/s)")
        layout["D"].set_xlim([0.0, 17.5])

        # set the plots for TEB control
        layout["E"].plot(plot_points[4], plot_points[0], f"-{CPLOT[4]}")
        layout["E"].get_yaxis().set_visible(False)
        layout["E"].set_xlabel("TEB-control\nlateral error\n(m/s)")
        layout["E"].set_xlim([0.0, 17.5])

        # set the plots for rear-wheel feedback control
        layout["F"].plot(plot_points[5], plot_points[0], f"-{CPLOT[5]}")
        layout["F"].get_yaxis().set_visible(False)
        layout["F"].set_xlabel("RW-feebback\nlateral error\n(m/s)")
        layout["F"].set_xlim([0.0, 17.5])

        plt.show()

# start as main function
if __name__ == "__main__":
    data = str(sys.argv[1])
    plot = PlotFromData(data)