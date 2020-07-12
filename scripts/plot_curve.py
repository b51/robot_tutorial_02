#!/usr/bin/env python3
#########################################################################
#
#              Author: b51
#                Mail: b51live@gmail.com
#            FileName: plot_curve.py
#
#          Created On: Fri Jul 10 22:59:46 2020
#     Licensed under The MIT License [see LICENSE for details]
#
#########################################################################

import numpy as np
import matplotlib.pyplot as plt
import sys


def f(m, c, x):
    return np.exp(m * x) + c

def main(argv):
    m1 = float(argv[2])
    c1 = float(argv[3])
    m2 = float(argv[4])
    c2 = float(argv[5])

    point_list = []
    with open(argv[1]) as data_file:
        for line in data_file:
            if line[0] == "#":
                continue
            point_list.append(np.fromstring(line, dtype=float, sep=' '))

    points = np.array(point_list)
    plt.plot(points.transpose()[0], points.transpose()[1], "bo")

    x = np.arange(0.0, 5.5, 0.01)
    s1 = f(m1, c1, x)
    s2 = f(m2, c2, x)
    line, = plt.plot(x, s1, color = "blue", lw=2)
    line, = plt.plot(x, s2, color = "red", lw=2)
    plt.show()


if __name__ == "__main__":
    main(sys.argv)
