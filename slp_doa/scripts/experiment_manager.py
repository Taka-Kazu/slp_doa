#!/usr/bin/env python

import subprocess
import time
import yaml
from tqdm import tqdm
import csv
import datetime

import rospkg
import rospy
from std_msgs.msg import Float64MultiArray

iteration = 5
finish_flag = False

class Result:
    def __init__(self):
        self.traveled_distance = 0
        self.traveled_time = 0
        self.collsion_count = 0
        self.min_distance = 0

    def output(self):
        return [self.traveled_distance, self.traveled_time, self.collsion_count, self.min_distance]

results = list()

def result_callback(msg):
    global finish_flag
    global results
    finish_flag = True
    print("traveled distance: " + str(msg.data[0]) + "[m]")
    print("traveled time: " + str(msg.data[1]) + "[s]")
    results[-1].traveled_distance = msg.data[0]
    results[-1].traveled_time = msg.data[1]

def collision_callback(msg):
    global results
    results[-1].collsion_count = int(msg.data[0])
    results[-1].min_distance = msg.data[1]

def main():
    global finish_flag
    global results

    processes = dict()
    with open('experiment.yaml') as f:
        processes = yaml.load(f)
    print(processes)

    doap_path = rospkg.RosPack().get_path("dynamic_obstacle_avoidance_planner")
    slp_doa_path = rospkg.RosPack().get_path("slp_doa")

    print("\033[032mstarting roscore ...\033[0m")
    roscore_p = subprocess.Popen(["roscore"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(2)

    rosparam_p = subprocess.Popen(["rosparam", "set", "use_sim_time", "true"])

    print("\033[032mstarting rviz ...\033[0m")
    rviz_p = subprocess.Popen(["rviz", "-d", slp_doa_path + "/config/slp_doa.rviz"])
    time.sleep(1)

    for i in tqdm(range(iteration)):
        rospy.init_node("experiment_manager", anonymous=True)
        r = rospy.Rate(50)
        result_sub = rospy.Subscriber('/result', Float64MultiArray, result_callback)
        collision_sub = rospy.Subscriber('/collision_info', Float64MultiArray, collision_callback)

        results.append(Result())

        running_processes = list()
        for process in processes:
            print("\033[032mstarting " + str(process) + " ...\033[0m")
            running_processes.append(subprocess.Popen(process))

        print("\033[032mwaiting for start ...\033[0m")
        time.sleep(5)
        running_processes.append(subprocess.Popen(["rosbag", "play", doap_path + "/bagfile/time.bag", "--clock"], stdout=subprocess.PIPE, stderr=subprocess.PIPE))

        while not rospy.is_shutdown():
            # wait for finishing experiment
            # print finish_flag
            if finish_flag:
                finish_flag = False
                break
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                print('time error')

        for p in running_processes:
            p.terminate()

        print(results[-1].output())

    rviz_p.terminate()

    # sometimes rosmaster is not killed properly
    # please run 'pkill rosmaster'
    roscore_p.terminate()

    print("\033[32m====================================\n|all experiments have been finished|\n====================================\033[0m")

    result_file_name = "result_" + datetime.datetime.now().strftime('%Y%m%d%H%M%S') + ".csv"
    with open(result_file_name , 'w') as f:
        writer = csv.writer(f)
        for result in results:
            writer.writerow(result.output())
    print("result is saved as " + result_file_name)

if __name__=="__main__":
    main()
