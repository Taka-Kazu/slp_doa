#ifndef __FAKE_LOCAL_COSTMAP_GENERATOR_H
#define __FAKE_LOCAL_COSTMAP_GENERATOR_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>

class FakeLocalCostmapGenerator
{
public:
    FakeLocalCostmapGenerator(void);
    void process(void);
    void obstacles_callback(const geometry_msgs::PoseArrayConstPtr&);

    int get_i_from_x(const nav_msgs::OccupancyGrid&, double x);
    int get_j_from_y(const nav_msgs::OccupancyGrid&, double y);
    int get_index(const nav_msgs::OccupancyGrid&, double x, double y);

private:
    double HZ;
    std::string ROBOT_FRAME;
    double RESOLUTION;
    double MAP_WIDTH;
    double COLLISION_RADIUS;
    bool ENABLE_STATIC_OBSTACLE;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Publisher map_pub;
    ros::Subscriber obstacles_sub;

    tf::TransformListener listener;
    geometry_msgs::PoseArray obstacles;
};

#endif// __FAKE_LOCAL_COSTMAP_GENERATOR_H
