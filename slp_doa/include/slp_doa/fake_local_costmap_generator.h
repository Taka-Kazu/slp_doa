#ifndef __FAKE_LOCAL_COSTMAP_GENERATOR_H
#define __FAKE_LOCAL_COSTMAP_GENERATOR_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>

class FakeLocalCostmapGenerator
{
public:
    FakeLocalCostmapGenerator(void);
    void process(void);

private:
    double HZ;
    std::string ROBOT_FRAME;
    double RESOLUTION;
    double MAP_WIDTH;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Publisher map_pub;
};

#endif// __FAKE_LOCAL_COSTMAP_GENERATOR_H
