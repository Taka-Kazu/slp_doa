#include "slp_doa/slp_doa.h"

SLPDOA::SLPDOA(void)
{
    std::cout << "======slp_doa ======" << std::endl;
    obstacle_pose_sub = nh.subscribe("/dynamic_obstacles", 1, &SLPDOA::obstacle_pose_callback, this);
}

void SLPDOA::obstacle_pose_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{

}

bool SLPDOA::check_collision(const nav_msgs::OccupancyGrid& local_costmap, const std::vector<Eigen::Vector3d>& trajectory)
{
    /*
     * if given trajectory is considered to collide with an obstacle, return true
     */
    double resolution = local_costmap.info.resolution;
    std::vector<Eigen::Vector3d> bresenhams_line;
    generate_bresemhams_line(trajectory, resolution, bresenhams_line);
    int size = bresenhams_line.size();
    for(int i=0;i<size;i++){
        int xi = round((bresenhams_line[i](0) - local_costmap.info.origin.position.x) / resolution);
        int yi = round((bresenhams_line[i](1) - local_costmap.info.origin.position.y) / resolution);
        //std::cout << xi << ", " << yi << std::endl;
        if(local_costmap.data[xi + local_costmap.info.width * yi] != 0){
            return true;
        }
    }
    return false;
}

bool SLPDOA::check_collision(const nav_msgs::OccupancyGrid& local_costmap, const std::vector<Eigen::Vector3d>& trajectory, double range)
{
    /*
     * if given trajectory is considered to collide with an obstacle, return true
     */
    double resolution = local_costmap.info.resolution;
    std::vector<Eigen::Vector3d> bresenhams_line;
    generate_bresemhams_line(trajectory, resolution, bresenhams_line);
    int size = bresenhams_line.size();
    for(int i=0;i<size;i++){
        int xi = round((bresenhams_line[i](0) - local_costmap.info.origin.position.x) / resolution);
        int yi = round((bresenhams_line[i](1) - local_costmap.info.origin.position.y) / resolution);
        //std::cout << xi << ", " << yi << std::endl;
        if(local_costmap.data[xi + local_costmap.info.width * yi] != 0){
            if(bresenhams_line[i].norm() > range){
                return false;
            }else{
                return true;
            }
        }
    }
    return false;
}
