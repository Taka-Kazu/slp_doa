#ifndef __SLP_DOA_H
#define __SLP_DOA_H

#include <ros/ros.h>

#include "state_lattice_planner/state_lattice_planner.h"
#include "dynamic_obstacle_avoidance_planner/obstacle_tracker_kf.h"

class SLPDOA: public StateLatticePlanner
{
public:
    SLPDOA(void);

    void obstacle_pose_callback(const geometry_msgs::PoseArrayConstPtr&);
    bool check_collision(const nav_msgs::OccupancyGrid&, const std::vector<Eigen::Vector3d>&);
    bool check_collision(const nav_msgs::OccupancyGrid&, const std::vector<Eigen::Vector3d>&, double);

private:
    double PREDICTION_TIME;
    int PREDICTION_STEP;
    double DT;

    ros::Publisher obstacles_predicted_path_pub;
    ros::Subscriber obstacle_pose_sub;

    geometry_msgs::PoseArray obstacle_pose;
    geometry_msgs::PoseArray obstacle_paths;
    ObstacleTrackerKF tracker;
};

#endif// __SLP_DOA_H
