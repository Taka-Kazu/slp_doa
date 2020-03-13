#include "slp_doa/fake_local_costmap_generator.h"

FakeLocalCostmapGenerator::FakeLocalCostmapGenerator(void)
:local_nh("~")
{
    local_nh.param("/dynamic_avoidance/ROBOT_FRAME", ROBOT_FRAME, {"/base_link"});
    local_nh.param("HZ", HZ, {20});
    local_nh.param("RESOLUTION", RESOLUTION, {0.05});
    local_nh.param("MAP_WIDTH", MAP_WIDTH, {20.0});
    local_nh.param("ENABLE_STATIC_OBSTACLE", ENABLE_STATIC_OBSTACLE, {false});
    local_nh.param("COLLISION_RADIUS", COLLISION_RADIUS, {0.6});

    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_costmap", 1);
    obstacles_sub = nh.subscribe("/dynamic_obstacles", 1, &FakeLocalCostmapGenerator::obstacles_callback, this);

    std::cout << "=== local_costmap ===" << std::endl;
    std::cout << "ROBOT_FRAME: " << ROBOT_FRAME  << std::endl;
    std::cout << "HZ: " << HZ  << std::endl;
    std::cout << "RESOLUTION: " << RESOLUTION  << std::endl;
    std::cout << "MAP_WIDTH: " << MAP_WIDTH  << std::endl;
    std::cout << "ENABLE_STATIC_OBSTACLE: " << ENABLE_STATIC_OBSTACLE  << std::endl;
    std::cout << "COLLISION_RADIUS: " << COLLISION_RADIUS  << std::endl;
}

void FakeLocalCostmapGenerator::obstacles_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
    geometry_msgs::PoseArray obstacles_ = *msg;
    try{
        for(auto& p : obstacles_.poses){
            geometry_msgs::PoseStamped p_;
            p_.header = obstacles_.header;
            p_.pose = p;
            listener.waitForTransform(ROBOT_FRAME, p_.header.frame_id, p_.header.stamp, ros::Duration(1.0));
            listener.transformPose(ROBOT_FRAME, p_, p_);
            p = p_.pose;
        }
        obstacles_.header.frame_id = ROBOT_FRAME;
        obstacles = obstacles_;
    }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
        return;
    }
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
        if(ENABLE_STATIC_OBSTACLE){
            std::cout << "obstacles: " << obstacles.poses.size() << std::endl;
            for(const auto& obstacle : obstacles.poses){
                for(double y=obstacle.position.y-COLLISION_RADIUS;y<obstacle.position.y+COLLISION_RADIUS;y+=RESOLUTION){
                    const double MAP_WIDTH_2 = MAP_WIDTH * 0.5;
                    if(-MAP_WIDTH_2 < y && y < MAP_WIDTH_2){
                        double x0 = obstacle.position.x - sqrt(COLLISION_RADIUS * COLLISION_RADIUS - (y - obstacle.position.y) * (y - obstacle.position.y));
                        double x1 = obstacle.position.x + sqrt(COLLISION_RADIUS * COLLISION_RADIUS - (y - obstacle.position.y) * (y - obstacle.position.y));
                        for(double x=x0;x<=x1;x+=RESOLUTION){
                            if(-MAP_WIDTH_2 < x && x < MAP_WIDTH_2){
                                int index = get_index(local_costmap, x, y);
                                if(0 <= index && index < map_size){
                                    local_costmap.data[index] = 100;
                                }
                            }
                        }
                    }
                }
            }
        }
        map_pub.publish(local_costmap);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

inline int FakeLocalCostmapGenerator::get_i_from_x(const nav_msgs::OccupancyGrid& map, double x)
{
    return floor((x - map.info.origin.position.x) / map.info.resolution + 0.5);
}

inline int FakeLocalCostmapGenerator::get_j_from_y(const nav_msgs::OccupancyGrid& map, double y)
{
    return floor((y - map.info.origin.position.y) / map.info.resolution + 0.5);
}

inline int FakeLocalCostmapGenerator::get_index(const nav_msgs::OccupancyGrid& map, double x, double y)
{
    return map.info.width * get_j_from_y(map, y) + get_i_from_x(map, x);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_local_costmap_generator");
    FakeLocalCostmapGenerator fake_local_costmap_generator;
    fake_local_costmap_generator.process();
    return 0;
}

