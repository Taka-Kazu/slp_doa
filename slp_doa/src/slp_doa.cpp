#include "slp_doa/slp_doa.h"

SLPDOA::SLPDOA(void)
{
    local_nh.param("PREDICTION_TIME", PREDICTION_TIME, {3.5});
    DT = 1.0 / HZ;
    PREDICTION_STEP = round(PREDICTION_TIME / DT);

    std::cout << " ======slp_doa ======" << std::endl;

    std::cout << "PREDICTION_TIME: " << PREDICTION_TIME << std::endl;
    std::cout << "PREDICTION_STEP: " << PREDICTION_STEP << std::endl;

    obstacles_predicted_path_pub = local_nh.advertise<geometry_msgs::PoseArray>("/obstacle_predicted_paths", 1);
    obstacle_pose_sub = nh.subscribe("/dynamic_obstacles", 1, &SLPDOA::obstacle_pose_callback, this);
}

SLPDOA::ObstacleStates::ObstacleStates(void)
{

}

SLPDOA::ObstacleStates::ObstacleStates(int prediction_step)
{
    pos.reserve(prediction_step);
    vel.reserve(prediction_step);
    cov.reserve(prediction_step);
}

double SLPDOA::ObstacleStates::calculate_probability(const Eigen::Vector2d& position, int step)
{
    if(step < 0){
        std::cout << "invalid time step" << std::endl;
        exit(-1);
    }
    Eigen::Vector2d mu = pos[step];
    Eigen::Matrix2d sigma = cov[step];
    double probability = 1. / (2 * M_PI * sqrt(sigma.determinant())) * std::exp(-0.5 * (position - mu).transpose() * sigma.inverse() * (position - mu));
    return probability;
}

void SLPDOA::obstacle_pose_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
    std::cout << "obstacle pose callback" << std::endl;
    obstacle_pose = *msg;
    tracker.set_obstacles_pose(obstacle_pose);
    int obs_num = tracker.obstacles.size();

    // prediction
    std::vector<ObstacleStates> obstacle_states_list;
    obstacle_states_list.reserve(obs_num);
    for(auto it=tracker.obstacles.begin();it!=tracker.obstacles.end();++it){
        ObstacleStates obstacle_states(PREDICTION_STEP);
        Obstacle obs = it->second;
        for(int i=0;i<PREDICTION_STEP;i++){
            obs.predict(DT);
            obstacle_states.pos.push_back(obs.x.segment(0, 2));
            obstacle_states.vel.push_back(obs.x.segment(2, 2));
            obstacle_states.cov.push_back(obs.p.block(0, 0, 2, 2));
        }
        obstacle_states_list.push_back(obstacle_states);
    }

    obstacle_paths.header = obstacle_pose.header;
    obstacle_paths.poses.clear();
    for(int i=0;i<obs_num;i++){
        std::cout << "obstacle " << i << ": "<< std::endl;
        std::cout << obstacle_states_list[i].pos[0] << std::endl;
        std::cout << obstacle_states_list[i].vel[0] << std::endl;
        for(int j=0;j<PREDICTION_STEP;j++){
            geometry_msgs::Pose p;
            p.position.x = obstacle_states_list[i].pos[i](0);
            p.position.y = obstacle_states_list[i].pos[i](1);
            p.orientation = tf::createQuaternionMsgFromYaw(atan2(obstacle_states_list[i].vel[i](1), obstacle_states_list[i].vel[i](0)));
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
