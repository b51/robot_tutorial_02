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

def main(argv):
    m = argv[2]
    c = argv[3]

    point_list = []
    with open(argv[1]) as data_file:
        for line in data_file:
            if line[0] == "#":
                continue
            point_list.append(np.fromstring(line, dtype=float, sep=' '))

    points = np.array(point_list)
    print(points.transpose()[0])
    plt.plot(points.transpose()[0], points.transpose()[1])
    plt.show()


if __name__ == "__main__":
    main(sys.argv)
