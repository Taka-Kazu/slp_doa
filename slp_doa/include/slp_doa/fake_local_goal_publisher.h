#ifndef __FAKE_LOCAL_GOAL_PUBLISHER_H
#define __FAKE_LOCAL_GOAL_PUBLISHER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

class FakeLocalGoalPublisher
{
public:
    FakeLocalGoalPublisher(void);
    void process(void);

private:
    double HZ;
    std::string WORLD_FRAME;
    std::string ROBOT_FRAME;
    // in world frame
    double GOAL_X;
    double GOAL_Y;
    double GOAL_YAW;
    double GOAL_DISTANCE;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Publisher goal_pub;
    tf::TransformListener listener;
};

#endif// __FAKE_LOCAL_GOAL_PUBLISHER_H
