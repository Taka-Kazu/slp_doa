#!/usr/bin/env python

import subprocess
import time
import yaml
from tqdm import tqdm
import csv
import datetime
import os

import rospkg
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion

iteration = 100
finish_flag = False

class Result:
    def __init__(self):
        self.traveled_distance = 0
        self.traveled_time = 0
        self.collision_count = 0
        self.min_distance = 0

    def output(self):
        return [self.traveled_distance, self.traveled_time, self.collision_count, self.min_distance]

results = list()
velocity_log = list()

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
    results[-1].collision_count = int(msg.data[0])
    results[-1].min_distance = msg.data[1]

last_v_time = None
def velocity_callback(msg):
    global velocity_log
    global last_v_time
    t = rospy.get_time()
    if last_v_time is None:
        velocity_log.append((msg.linear.x, msg.angular.z, 0, 0))
    elif t == last_v_time:
        return
    else:
        # dt = t - last_v_time
        dt = 0.05
        a = (msg.linear.x - velocity_log[-1][0]) / dt
        aw = (msg.angular.z - velocity_log[-1][1]) / dt
        velocity_log.append((msg.linear.x, msg.angular.z, a, aw))
    last_v_time = t

def main():
    timestamp = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
    global finish_flag
    global results
    global velocity_log
    global last_v_time

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

    # print("\033[032mstarting rviz ...\033[0m")
    # rviz_p = subprocess.Popen(["rviz", "-d", slp_doa_path + "/config/slp_doa.rviz"])
    time.sleep(1)

    for i in tqdm(range(iteration)):
        rospy.init_node("experiment_manager", anonymous=True)
        rospy.set_param("/sfm_obstacle_simulator/SEED", 10000 + i)
        r = rospy.Rate(50)
        result_sub = rospy.Subscriber('/result', Float64MultiArray, result_callback)
        vel_sub = rospy.Subscriber('/velocity/debug', Twist, velocity_callback)
        collision_sub = rospy.Subscriber('/collision_info', Float64MultiArray, collision_callback)

        results.append(Result())
        del velocity_log[:]
        last_v_time = None

        running_processes = list()
        for process in processes:
            print("\033[032mstarting " + str(process) + " ...\033[0m")
            running_processes.append(subprocess.Popen(process, stdout=None, stderr=None))

        print("\033[032mwaiting for start ...\033[0m")
        time.sleep(5)
        running_processes.append(subprocess.Popen(["rosbag", "play", doap_path + "/bagfile/time.bag", "--clock"], stdout=subprocess.PIPE, stderr=subprocess.PIPE))

        sim_start_time = rospy.get_time()
        while sim_start_time == 0.0:
            sim_start_time = rospy.get_time()

        while not rospy.is_shutdown():
            process_time = rospy.get_time() - sim_start_time
            if process_time > 60.:
                print("\033[031mtime over!!!\033[0m")
                finish_flag = False
                del results[-1]
                break

            # wait for finishing experiment
            # print finish_flag
            if finish_flag:
                finish_flag = False
                break
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                time.sleep(0.1)
                print('time error')

        for p in running_processes:
            p.terminate()

        print(results[-1].output())
        velocity_log_dir_name = "results/velocity_log_" + str(iteration) + "_" + timestamp
        try:
            os.makedirs(velocity_log_dir_name)
            # os.symlink(velocity_log_dir_name, "velocity_log_latest")
        except OSError:
            pass
        with open(velocity_log_dir_name + "/" + str(i) + ".csv" , 'w') as f:
            writer = csv.writer(f)
            for v in velocity_log:
                writer.writerow(v)

        time.sleep(1)
        subprocess.call("yes | rosnode cleanup", shell=True)

    # rviz_p.terminate()

    # sometimes rosmaster is not killed properly
    # please run 'pkill rosmaster'
    roscore_p.terminate()

    print("\033[32m====================================\n|all experiments have been finished|\n====================================\033[0m")

    result_file_name = "results/result_" + str(iteration) + "_" + timestamp + ".csv"
    with open(result_file_name , 'w') as f:
        writer = csv.writer(f)
        for result in results:
            writer.writerow(result.output())
    # os.symlink(result_file_name, "latest")
    print("result is saved as " + result_file_name)

if __name__=="__main__":
    main()
