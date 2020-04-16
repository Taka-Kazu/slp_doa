#ifndef __SLP_DOA_ROS_H
#define __SLP_DOA_ROS_H

#include <ros/ros.h>

#include "state_lattice_planner/state_lattice_planner_ros.h"
#include "slp_doa/slp_doa.h"

class SLPDOAROS
{
public:
    SLPDOAROS(void);

    void local_goal_callback(const geometry_msgs::PoseStampedConstPtr&);
    void local_map_callback(const nav_msgs::OccupancyGridConstPtr&);
    void odom_callback(const nav_msgs::OdometryConstPtr&);
    void target_velocity_callback(const geometry_msgs::TwistConstPtr&);
    void obstacle_pose_callback(const geometry_msgs::PoseArrayConstPtr&);
    SLPDOA::ProbabilityWithTimeStep get_collision_probability(const nav_msgs::OccupancyGrid&, const std::vector<Eigen::Vector3d>&, std::vector<double>&);
    // void generate_probability_map(unsigned int);
    void visualize_trajectories_with_probability(const std::vector<MotionModelDiffDrive::Trajectory>&, const double, const double, const double, const int, const ros::Publisher&, const std::vector<std::vector<double> >&, const std::vector<unsigned int>&);
    void process(void);
    template<typename TYPE>
    void get_obstacle_map(const nav_msgs::OccupancyGrid&, state_lattice_planner::ObstacleMap<TYPE>&);

protected:
    void visualize_trajectories(const std::vector<MotionModelDiffDrive::Trajectory>&, const double, const double, const double, const int, const ros::Publisher&);
    void visualize_trajectory(const MotionModelDiffDrive::Trajectory&, const double, const double, const double, const ros::Publisher&);

    double HZ;
    std::string ROBOT_FRAME;
    int N_P;
    int N_H;
    int N_S;
    double MAX_ALPHA;
    double MAX_PSI;
    double MAX_ACCELERATION;
    double TARGET_VELOCITY;
    std::string LOOKUP_TABLE_FILE_NAME;
    int MAX_ITERATION;
    double OPTIMIZATION_TOLERANCE;
    double MAX_YAWRATE;
    double MAX_D_YAWRATE;
    double MAX_WHEEL_ANGULAR_VELOCITY;
    double WHEEL_RADIUS;
    double TREAD;
    double IGNORABLE_OBSTACLE_RANGE;
    bool VERBOSE;
    int CONTROL_DELAY;
    double TURN_DIRECTION_THRESHOLD;
    bool ENABLE_SHARP_TRAJECTORY;
    bool ENABLE_CONTROL_SPACE_SAMPLING;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher velocity_pub;
    ros::Publisher candidate_trajectories_pub;
    ros::Publisher candidate_trajectories_no_collision_pub;
    ros::Publisher selected_trajectory_pub;
    ros::Subscriber local_map_sub;
    ros::Subscriber local_goal_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber target_velocity_sub;
    tf::TransformListener listener;
    geometry_msgs::PoseStamped local_goal;
    nav_msgs::OccupancyGrid local_map;
    geometry_msgs::Twist current_velocity;
    bool local_goal_subscribed;
    bool local_map_updated;
    bool odom_updated;

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
