#include "slp_doa/fake_local_costmap_generator.h"

FakeLocalCostmapGenerator::FakeLocalCostmapGenerator(void)
:local_nh("~")
{
    local_nh.param("/dynamic_avoidance/ROBOT_FRAME", ROBOT_FRAME, {"/base_link"});
    local_nh.param("HZ", HZ, {10});
    local_nh.param("RESOLUTION", RESOLUTION, {0.05});
    local_nh.param("MAP_WIDTH", MAP_WIDTH, {20.0});

    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_costmap", 1);

    std::cout << "=== local_costmap ===" << std::endl;
    std::cout << "ROBOT_FRAME: " << ROBOT_FRAME  << std::endl;
    std::cout << "HZ: " << HZ  << std::endl;
    std::cout << "RESOLUTION: " << RESOLUTION  << std::endl;
    std::cout << "MAP_WIDTH: " << MAP_WIDTH  << std::endl;
}

void FakeLocalCostmapGenerator::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        std::cout << "=== fake_local_costmap ===" << std::endl;
        nav_msgs::OccupancyGrid local_costmap;
        local_costmap.header.frame_id = ROBOT_FRAME;
        local_costmap.header.stamp = ros::Time::now();
        local_costmap.info.resolution = RESOLUTION;
        local_costmap.info.width = MAP_WIDTH / RESOLUTION;
        local_costmap.info.height = MAP_WIDTH / RESOLUTION;
        local_costmap.info.origin.position.x = -MAP_WIDTH * 0.5;
        local_costmap.info.origin.position.y = -MAP_WIDTH * 0.5;
        local_costmap.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
        int map_size = local_costmap.info.width * local_costmap.info.height;
        local_costmap.data.resize(map_size);
        for(auto& data : local_costmap.data){
            data = 0;
        }
        map_pub.publish(local_costmap);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_local_costmap_generator");
    FakeLocalCostmapGenerator fake_local_costmap_generator;
    fake_local_costmap_generator.process();
    return 0;
}

