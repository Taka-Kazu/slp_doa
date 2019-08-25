#include "slp_doa/fake_local_goal_publisher.h"

FakeLocalGoalPublisher::FakeLocalGoalPublisher(void)
:local_nh("~")
{
    local_nh.param("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME, {"/map"});
    local_nh.param("HZ", HZ, {10});
    local_nh.param("GOAL_X", GOAL_X, {10.0});
    local_nh.param("GOAL_Y", GOAL_Y, {0.0});
    local_nh.param("GOAL_YAW", GOAL_YAW, {0.0});

    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);

    std::cout << "=== fake_local_goal_publisher ===" << std::endl;
    std::cout << "WORLD_FRAME: " << WORLD_FRAME  << std::endl;
    std::cout << "HZ: " << HZ  << std::endl;
}

void FakeLocalGoalPublisher::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        std::cout << "=== fake_local_goal_publisher ===" << std::endl;
        geometry_msgs::PoseStamped local_goal;
        local_goal.header.frame_id = WORLD_FRAME;
        local_goal.header.stamp = ros::Time::now();
        local_goal.pose.position.x = GOAL_X;
        local_goal.pose.position.y = GOAL_Y;
        local_goal.pose.orientation = tf::createQuaternionMsgFromYaw(GOAL_YAW);
        goal_pub.publish(local_goal);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_local_goal_publisher");
    FakeLocalGoalPublisher fake_local_goal_publisher;
    fake_local_goal_publisher.process();
    return 0;
}

