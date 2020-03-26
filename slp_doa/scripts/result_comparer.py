#!/usr/bin/env python

import csv
import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

def main():
    parser = argparse.ArgumentParser(description='read result from csv')
    parser.add_argument('file_name_0', metavar='file_name_0', type=str, help='input file name')
    parser.add_argument('file_name_1', metavar='file_name_1', type=str, help='input file name')
    args = parser.parse_args()

    result0 = np.loadtxt(args.file_name_0, delimiter=',')
    result1 = np.loadtxt(args.file_name_1, delimiter=',')
    plt.rcParams['font.family'] = 'Times new Roman'

    method_name = "cadrl"

    fig = plt.figure()
    ax0 = fig.add_subplot(2, 2, 1)
    ax1 = fig.add_subplot(2, 2, 2)
    ax2 = fig.add_subplot(2, 2, 3)
    ax3 = fig.add_subplot(2, 2, 4)
    bp0 = ax0.boxplot((result0[:, 0], result1[:, 0]), whis="range", showmeans=True, meanline=True)
    ax0.set_xticklabels(['Proposed method', method_name])
    ax0.set_title('(a) Travel distance[m]', y=-0.35, fontsize=10)
    ax0.set_ylim(0, 60)
    ax0.set_yticks(np.arange(0, ax0.get_ylim()[1]+10, 10))
    ax0.grid()
    print("travel distance")
    print("P. M. mean: " + str(result0[:, 0].mean()) + "[m]")
    print("P. M. mean: " + str(result0[result0[:, 2]==0, 0].mean()) + "[m]")
    print("P. M. median: " + str(np.median(result0[:, 0])) + "[m]")
    print("P. M. stddev: " + str(result0[:, 0].std()))
    print(method_name + " mean: " + str(result1[:, 0].mean()) + "[m]")
    print(method_name + " mean: " + str(result1[result1[:, 2]==0, 0].mean()) + "[m]")
    print(method_name + " median: " + str(np.median(result1[:, 0])) + "[m]")
    print(method_name + " stddev: " + str(result1[:, 0].std()))
    print(stats.ttest_ind(result0[:, 0], result1[:, 0], equal_var=False))
    print(stats.ttest_ind(result0[result0[:, 2]==0, 0], result1[result1[:, 2]==0, 0], equal_var=False))

    bp1 = ax1.boxplot((result0[:, 1], result1[:, 1]), whis="range", showmeans=True, meanline=True)
    ax1.set_xticklabels(['Proposed method', method_name])
    ax1.set_title('(b) Travel time[s]', y=-0.35, fontsize=10)
    ax1.set_ylim(0, 60)
    ax1.set_yticks(np.arange(0, ax1.get_ylim()[1]+10, 10))
    ax1.grid()
    print("travel time")
    print("P. M. mean: " + str(result0[:, 1].mean()) + "[s]")
    print("P. M. mean: " + str(result0[result0[:, 2]==0, 1].mean()) + "[s]")
    print("P. M. median: " + str(np.median(result0[:, 1])) + "[s]")
    print("P. M. stddev: " + str(result0[:, 1].std()))
    print(method_name + " mean: " + str(result1[:, 1].mean()) + "[s]")
    print(method_name + " mean: " + str(result1[result1[:, 2]==0, 1].mean()) + "[s]")
    print(method_name + " median: " + str(np.median(result1[:, 1])) + "[s]")
    print(method_name + " stddev: " + str(result1[:, 1].std()))
    print(stats.ttest_ind(result0[:, 1], result1[:, 1], equal_var=False))
    print(stats.ttest_ind(result0[result0[:, 2]==0, 1], result1[result1[:, 2]==0, 1], equal_var=False))

    bp2 = ax2.boxplot((result0[:, 2], result1[:, 2]), whis="range", showmeans=True, meanline=True)
    ax2.set_xticklabels(['Proposed method', method_name])
    ax2.set_title('(c) Number of collisions', y=-0.35, fontsize=10)
    ax2.set_ylim(0, 6)
    ax2.set_yticks(np.arange(0, ax2.get_ylim()[1]+1, 1))
    ax2.grid()
    print("collision count")
    print("P. M. mean: " + str(result0[:, 2].mean()))
    print("P. M. median: " + str(np.median(result0[:, 2])))
    print("P. M. stddev: " + str(result0[:, 2].std()))
    print("P. M. c. rate: " + str(np.where(result0[:, 2] > 0)[0].shape[0] / float(result0[:, 2].shape[0])))
    print(method_name + " mean: " + str(result1[:, 2].mean()))
    print(method_name + " median: " + str(np.median(result1[:, 2])))
    print(method_name + " stddev: " + str(result1[:, 2].std()))
    print(method_name + " c. rate: " + str(np.where(result1[:, 2] > 0)[0].shape[0] / float(result1[:, 2].shape[0])))
    print(stats.ttest_ind(result0[:, 2], result1[:, 2], equal_var=False))

    bp3 = ax3.boxplot((result0[:, 3], result1[:, 3]), whis="range", showmeans=True, meanline=True)
    ax3.set_xticklabels(['Proposed method', method_name])
    ax3.set_title('(d) Minimum distance[m]', y=-0.35, fontsize=10)
    ax3.set_ylim(0, 1.5)
    ax3.set_yticks(np.arange(0, ax3.get_ylim()[1]+0.5, 0.5))
    ax3.axhline(0.6, c="r")
    ax3.text(2.5, 0.6, "$R_{col}$", size=10, color="red")
    ax3.grid()
    print("min distance")
    print("P. M. mean: " + str(result0[:, 3].mean()) + "[m]")
    print("P. M. mean: " + str(result0[result0[:, 2]==0, 3].mean()) + "[m]")
    print("P. M. median: " + str(np.median(result0[:, 3])) + "[m]")
    print("P. M. stddev: " + str(result0[:, 3].std()))
    print(method_name + "mean: " + str(result1[:, 3].mean()) + "[m]")
    print(method_name + " mean: " + str(result1[result1[:, 2]==0, 3].mean()) + "[m]")
    print(method_name + "median: " + str(np.median(result1[:, 3])) + "[m]")
    print(method_name + " stddev: " + str(result1[:, 3].std()))
    print(stats.ttest_ind(result0[:, 3], result1[:, 3], equal_var=False))
    print(stats.ttest_ind(result0[result0[:, 2]==0, 3], result1[result1[:, 2]==0, 3], equal_var=False))

    plt.subplots_adjust(wspace=0.4, hspace=0.4)
    plt.show()

if __name__=="__main__":
    main()
