#ifndef __FAKE_LOCAL_GOAL_PUBLISHER_H
#define __FAKE_LOCAL_GOAL_PUBLISHER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

class FakeLocalGoalPublisher
{
public:
    FakeLocalGoalPublisher(void);
    void process(void);

private:
    double HZ;
    std::string WORLD_FRAME;
    double GOAL_X;
    double GOAL_Y;
    double GOAL_YAW;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Publisher goal_pub;
};

#endif// __FAKE_LOCAL_GOAL_PUBLISHER_H
