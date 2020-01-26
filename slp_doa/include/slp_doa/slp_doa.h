#ifndef __SLP_DOA_H
#define __SLP_DOA_H

#include <ros/ros.h>

#include "state_lattice_planner/state_lattice_planner.h"
#include "dynamic_obstacle_avoidance_planner/obstacle_tracker_kf.h"

#include <omp.h>

class SLPDOA: public StateLatticePlanner
{
public:
    SLPDOA(void);

    class ObstacleStates
    {
    public:
        ObstacleStates(void);
        ObstacleStates(int, double);

        double calculate_probability(const Eigen::Vector2d&, int) const;
        double square(double) const;

        std::vector<Eigen::Vector2d> pos;
        std::vector<Eigen::Vector2d> vel;
        std::vector<Eigen::Matrix2d> vcov;
        double resolution;
    };

    class ProbabilityWithTimeStep
    {
    public:
        ProbabilityWithTimeStep(void);
        ProbabilityWithTimeStep(unsigned int, double);

        unsigned int time_step;
        double probability;
    };


    void obstacle_pose_callback(const geometry_msgs::PoseArrayConstPtr&);
    ProbabilityWithTimeStep get_collision_probability(const nav_msgs::OccupancyGrid&, const std::vector<Eigen::Vector3d>&, std::vector<double>&);
    void generate_probability_map(unsigned int);
    void visualize_trajectories_with_probability(const std::vector<MotionModelDiffDrive::Trajectory>&, const double, const double, const double, const int, const ros::Publisher&, const std::vector<std::vector<double> >&, const std::vector<unsigned int>&);
    void process(void);

private:

    double PREDICTION_TIME;
    unsigned int PREDICTION_STEP;
    double DT;
    double COLLISION_PROBABILITY_THRESHOLD;
    std::string WORLD_FRAME;

    ros::Publisher obstacles_predicted_path_pub;
    ros::Publisher probability_map_pub;
    ros::Subscriber obstacle_pose_sub;

    geometry_msgs::PoseArray obstacle_pose;
    geometry_msgs::PoseArray obstacle_paths;
    ObstacleTrackerKF tracker;
    std::vector<ObstacleStates> obstacle_states_list;
};

#endif// __SLP_DOA_H
