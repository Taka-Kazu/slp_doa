#include "slp_doa/slp_doa.h"

SLPDOA::SLPDOA(void)
{
    local_nh.param("PREDICTION_TIME", PREDICTION_TIME, {3.5});
    DT = HZ;
    PREDICTION_STEP = round(PREDICTION_TIME / DT) + 1;

    std::cout << " ======slp_doa ======" << std::endl;

    std::cout << "PREDICTION_TIME" << PREDICTION_TIME << std::endl;
    std::cout << "PREDICTION_STEP" << PREDICTION_STEP << std::endl;

    obstacles_predicted_path_pub = local_nh.advertise<geometry_msgs::PoseArray>("/obstacle_predicted_paths", 1);
    obstacle_pose_sub = nh.subscribe("/dynamic_obstacles", 1, &SLPDOA::obstacle_pose_callback, this);
}

void SLPDOA::obstacle_pose_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
    std::cout << "obstacle pose callback" << std::endl;
    obstacle_pose = *msg;
    tracker.set_obstacles_pose(obstacle_pose);
    std::vector<Eigen::Vector3d> poses;
    tracker.get_poses(poses);
    int obs_num = poses.size();
    std::vector<Eigen::Vector3d> velocities;
    tracker.get_velocities(velocities);

    obstacle_paths.header = obstacle_pose.header;
    obstacle_paths.poses.clear();
    for(int i=0;i<obs_num;i++){
        std::cout << "obstacle " << i << ": "<< std::endl;
        std::cout << poses[i] << std::endl;
        std::cout << velocities[i] << std::endl;
        geometry_msgs::Pose p;
        p.position.x = poses[i](0);
        p.position.y = poses[i](1);
        p.orientation = tf::createQuaternionMsgFromYaw(poses[i](2));
        obstacle_paths.poses.push_back(p);
        for(int j=1;j<PREDICTION_STEP;j++){
            p.position.x += velocities[i](0) * DT;
            p.position.y += velocities[i](1) * DT;
            double yaw = tf::getYaw(p.orientation);
            yaw += velocities[i](2) * DT;
            yaw = atan2(sin(yaw), cos(yaw));
            p.orientation = tf::createQuaternionMsgFromYaw(yaw);
            // std::cout << p << std::endl;
            obstacle_paths.poses.push_back(p);
        }
    }
    std::cout << "obs num: " << obs_num << std::endl;
    std::cout << "obs path num: " << obstacle_paths.poses.size() << std::endl;
    obstacles_predicted_path_pub.publish(obstacle_paths);
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
