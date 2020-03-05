#!/usr/bin/env python

import csv
import argparse
import numpy as np
import matplotlib.pyplot as plt

def main():
    parser = argparse.ArgumentParser(description='read result from csv')
    parser.add_argument('file_name', metavar='file_name', type=str, help='input file name')
    args = parser.parse_args()
    print(args.file_name)

    results = np.loadtxt(args.file_name, delimiter=',')
    print(results)

    print("ave. traveled distance: " + str(results[:, 0].mean()) + "[m]")
    print("ave. traveled time: " + str(results[:, 1].mean()) + "[s]")
    print("ave. collision count: " + str(results[:, 2].mean()))

    fig, ax = plt.subplots()
    bp = ax.boxplot(results)
    ax.set_xticklabels(['traveled distance', 'traveled time', 'collision count', 'min distance'])
    plt.grid()
    plt.show()

if __name__=="__main__":
    main()
