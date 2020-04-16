#ifndef __SLP_DOA_ROS_H
#define __SLP_DOA_ROS_H

#include <ros/ros.h>

#include "state_lattice_planner/state_lattice_planner_ros.h"
#include "slp_doa/slp_doa.h"

class SLPDOAROS
{
public:
    SLPDOAROS(void);

    void obstacle_pose_callback(const geometry_msgs::PoseArrayConstPtr&);
    SLPDOA::ProbabilityWithTimeStep get_collision_probability(const nav_msgs::OccupancyGrid&, const std::vector<Eigen::Vector3d>&, std::vector<double>&);
    // void generate_probability_map(unsigned int);
    void visualize_trajectories_with_probability(const std::vector<MotionModelDiffDrive::Trajectory>&, const double, const double, const double, const int, const ros::Publisher&, const std::vector<std::vector<double> >&, const std::vector<unsigned int>&);
    void process(void);
    template<typename TYPE>
    void get_obstacle_map(const nav_msgs::OccupancyGrid&, state_lattice_planner::ObstacleMap<TYPE>&);

private:

    double PREDICTION_TIME;
    unsigned int PREDICTION_STEP;
    double DT;
    double COLLISION_PROBABILITY_THRESHOLD;
    std::string WORLD_FRAME;

    ros::Publisher obstacles_predicted_path_pub;
    ros::Publisher probability_map_pub;
    ros::Subscriber obstacle_pose_sub;

    SLPDOA slp_doa_planner;

    // geometry_msgs::PoseArray obstacle_pose;
    // geometry_msgs::PoseArray obstacle_paths;
    // ObstacleTrackerKF tracker;
    // std::vector<ObstacleStates> obstacle_states_list;
};

#endif// __SLP_DOA_ROS_H
