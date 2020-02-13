#include "slp_doa/fake_local_goal_publisher.h"

FakeLocalGoalPublisher::FakeLocalGoalPublisher(void)
:local_nh("~")
{
    local_nh.param("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME, {"/map"});
    local_nh.param("/dynamic_avoidance/ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
    local_nh.param("HZ", HZ, {10});
    local_nh.param("GOAL_X", GOAL_X, {10.0});
    local_nh.param("GOAL_Y", GOAL_Y, {0.0});
    local_nh.param("GOAL_YAW", GOAL_YAW, {0.0});
    local_nh.param("GOAL_DISTANCE", GOAL_DISTANCE, {5.0});

    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);

    std::cout << "=== fake_local_goal_publisher ===" << std::endl;
    std::cout << "WORLD_FRAME: " << WORLD_FRAME  << std::endl;
    std::cout << "ROBOT_FRAME: " << ROBOT_FRAME  << std::endl;
    std::cout << "HZ: " << HZ  << std::endl;
}

void FakeLocalGoalPublisher::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        std::cout << "=== fake_local_goal_publisher ===" << std::endl;
        try{
            geometry_msgs::PoseStamped local_goal;
            local_goal.header.frame_id = WORLD_FRAME;
            local_goal.header.stamp = ros::Time(0);
            local_goal.pose.position.x = GOAL_X;
            local_goal.pose.position.y = GOAL_Y;
            local_goal.pose.orientation = tf::createQuaternionMsgFromYaw(GOAL_YAW);
            listener.transformPose(ROBOT_FRAME, local_goal, local_goal);
            Eigen::Vector2d goal_vector(local_goal.pose.position.x, local_goal.pose.position.y);
            goal_vector = std::min(GOAL_DISTANCE, goal_vector.norm()) * goal_vector.normalized();
            local_goal.pose.position.x = goal_vector(0);
            local_goal.pose.position.y = goal_vector(1);
            goal_pub.publish(local_goal);
        }catch(tf::TransformException& ex){
            std::cout << ex.what() << std::endl;
        }
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

