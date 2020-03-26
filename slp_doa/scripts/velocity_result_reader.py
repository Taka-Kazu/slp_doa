#!/usr/bin/env python

import csv
import argparse
import numpy as np
import matplotlib.pyplot as plt
import os

def main():
    parser = argparse.ArgumentParser(description='read result')
    parser.add_argument('dir_name', metavar='dir_name', type=str, help='input dir name')
    args = parser.parse_args()
    print(args.dir_name)

    files = os.listdir(args.dir_name)
    # print(files)

    # results = list()
    results = None
    for name in files:
        result = np.loadtxt(args.dir_name + name, delimiter=',')
        if result.shape[0] is 0:
            print("error in ", name)
            continue
        if results is not None:
            results = np.vstack((results, result))
        else:
            results = result
        # results.append(result)

    # results = np.loadtxt(args.dir_name, delimiter=',')
    print("v mean: " + str(results[:, 0][results[:, 0] >= 0].mean()))
    print("v stddev: " + str(results[:, 0][results[:, 0] >= 0].std()))
    print("v min: " + str(results[:, 0][results[:, 0] >= 0].min()))
    print("v max: " + str(results[:, 0][results[:, 0] >= 0].max()))
    print("w mean: " + str(results[:, 1].mean()))
    print("w stddev: " + str(results[:, 1].std()))
    print("w min: " + str(results[:, 1].min()))
    print("w max: " + str(results[:, 1].max()))
    print("a mean: " + str(results[:, 2].mean()))
    print("a stddev: " + str(results[:, 2].std()))
    print("a min: " + str(results[:, 2].min()))
    print("a max: " + str(results[:, 2].max()))
    print("aw mean: " + str(results[:, 3].mean()))
    print("aw stddev: " + str(results[:, 3].std()))
    print("aw min: " + str(results[:, 3].min()))
    print("aw max: " + str(results[:, 3].max()))

    # print("ave. traveled distance: " + str(results[:, 0].mean()) + "[m]")
    # print("ave. traveled time: " + str(results[:, 1].mean()) + "[s]")
    # print("ave. collision count: " + str(results[:, 2].mean()))

    # fig, ax = plt.subplots()
    # bp = ax.boxplot(results)
    # ax.set_xticklabels(['traveled distance', 'traveled time', 'collision count', 'min distance'])
    # plt.grid()
    # plt.show()

if __name__=="__main__":
    main()
