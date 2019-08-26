#include "slp_doa/slp_doa.h"

SLPDOA::SLPDOA(void)
{
    local_nh.param("PREDICTION_TIME", PREDICTION_TIME, {3.5});
    local_nh.param("COLLISION_PROBABILITY_THRESHOLD", COLLISION_PROBABILITY_THRESHOLD, {0.1});
    local_nh.param("WORLD_FRAME", WORLD_FRAME, {"map"});
    DT = 1.0 / HZ;
    PREDICTION_STEP = round(PREDICTION_TIME / DT);

    if(COLLISION_PROBABILITY_THRESHOLD < 0.0 || 1.0 < COLLISION_PROBABILITY_THRESHOLD){
        std::cout << "\033[31mthe range of COLLISION_PROBABILITY_THRESHOLD must be between 0.0 and 1.0\033[0m" << std::endl;
        exit(-1);
    }

    std::cout << "====== slp_doa ======" << std::endl;

    std::cout << "PREDICTION_TIME: " << PREDICTION_TIME << std::endl;
    std::cout << "PREDICTION_STEP: " << PREDICTION_STEP << std::endl;
    std::cout << "COLLISION_PROBABILITY_THRESHOLD: " << COLLISION_PROBABILITY_THRESHOLD << std::endl;
    std::cout << "WORLD_FRAME: " << WORLD_FRAME << std::endl;

    obstacles_predicted_path_pub = local_nh.advertise<geometry_msgs::PoseArray>("/obstacle_predicted_paths", 1);
    obstacle_pose_sub = nh.subscribe("/dynamic_obstacles", 1, &SLPDOA::obstacle_pose_callback, this);
    std::cout << std::endl;
}

SLPDOA::ObstacleStates::ObstacleStates(void)
{
    resolution = 0.0;
}

SLPDOA::ObstacleStates::ObstacleStates(int prediction_step, double resolution_)
{
    pos.reserve(prediction_step);
    vel.reserve(prediction_step);
    vcov.reserve(prediction_step);
    resolution = resolution_;
}

double SLPDOA::ObstacleStates::calculate_probability(const Eigen::Vector2d& position, int step) const
{
    if(step < 0){
        std::cout << "invalid time step" << std::endl;
        exit(-1);
    }
    Eigen::Vector2d mu = pos[step];
    Eigen::Matrix2d sigma = vcov[step];
    double coeff = 1. / (2 * M_PI * sqrt(sigma.determinant()));
    double avoidance_radius = 0.6;
    double probability = 0;
    for(double x=position(0)-avoidance_radius;x<position(0)+avoidance_radius;x+=resolution){
        for(double y=position(1)-avoidance_radius;y<position(1)+avoidance_radius;y+=resolution){
            Eigen::Vector2d p(x, y);
            double probability_ =  coeff * std::exp(-0.5 * (p - mu).transpose() * sigma.inverse() * (p - mu));
            probability += probability_;
        }
    }
    probability *= coeff * resolution * resolution;
    if((position - mu).norm() < 0.6){
        std::cout << "t=" << step << std::endl;
        std::cout << "position:\n" << position.transpose() << std::endl;
        std::cout << "mu:\n" << mu.transpose() << std::endl;
        std::cout << "sigma:\n" << sigma << std::endl;
        std::cout << "prob: " << probability << std::endl;
    }
    if(std::isnan(probability) or std::isinf(probability)){
        std::cout << probability << std::endl;
        exit(-1);
    }
    return probability;
}

void SLPDOA::obstacle_pose_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
    std::cout << "obstacle pose callback" << std::endl;
    obstacle_pose = *msg;
    try{
        for(auto& p : obstacle_pose.poses){
            geometry_msgs::PoseStamped p_;
            p_.header = obstacle_pose.header;
            p_.pose = p;
            listener.transformPose(WORLD_FRAME, p_, p_);
            p = p_.pose;
        }
        obstacle_pose.header.frame_id = WORLD_FRAME;
    }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
        return;
    }

    tracker.set_obstacles_pose(obstacle_pose);
    int obs_num = tracker.obstacles.size();

    // prediction
    obstacle_states_list.clear();
    obstacle_states_list.reserve(obs_num);
    for(auto it=tracker.obstacles.begin();it!=tracker.obstacles.end();++it){
        ObstacleStates obstacle_states(PREDICTION_STEP, local_map.info.resolution);
        Obstacle obs = it->second;
        for(int i=0;i<PREDICTION_STEP;i++){
            obs.predict(DT);
            obstacle_states.pos.push_back(obs.x.segment(0, 2));
            obstacle_states.vel.push_back(obs.x.segment(2, 2));
            obstacle_states.vcov.push_back(obs.p.block(0, 0, 2, 2));
        }
        obstacle_states_list.push_back(obstacle_states);
    }

    // for debug
    obstacle_paths.header = obstacle_pose.header;
    obstacle_paths.poses.clear();
    for(int i=0;i<obs_num;i++){
        std::cout << "obstacle " << i << ": "<< std::endl;
        std::cout << obstacle_states_list[i].pos[0] << std::endl;
        std::cout << obstacle_states_list[i].vel[0] << std::endl;
        for(int j=0;j<PREDICTION_STEP;j++){
            geometry_msgs::Pose p;
            p.position.x = obstacle_states_list[i].pos[j](0);
            p.position.y = obstacle_states_list[i].pos[j](1);
            p.orientation = tf::createQuaternionMsgFromYaw(atan2(obstacle_states_list[i].vel[j](1), obstacle_states_list[i].vel[j](0)));
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
    Eigen::Affine3d affine;
    try{
        tf::StampedTransform stamped_transform;
        listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), stamped_transform);
        Eigen::Translation<double, 3> trans(stamped_transform.getOrigin().x(), stamped_transform.getOrigin().y(), stamped_transform.getOrigin().z());
        Eigen::Quaterniond q(stamped_transform.getRotation().w(), stamped_transform.getRotation().x(), stamped_transform.getRotation().y(), stamped_transform.getRotation().z());
        affine = trans * q;
    }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
        return true;
    }
    double resolution = local_costmap.info.resolution;
    int size = trajectory.size();
    for(int i=0;i<size;i++){
        int xi = round((trajectory[i](0) - local_costmap.info.origin.position.x) / resolution);
        int yi = round((trajectory[i](1) - local_costmap.info.origin.position.y) / resolution);
        if(local_costmap.data[xi + local_costmap.info.width * yi] != 0){
            return true;
        }else{
            if(i < PREDICTION_STEP){
                double no_collision_probability = 1;
                for(const auto& obs : obstacle_states_list){
                    no_collision_probability *= (1 - obs.calculate_probability((affine * trajectory[i]).segment(0, 2), i));
                }
                double collision_probability = 1 - no_collision_probability;
                if(collision_probability > COLLISION_PROBABILITY_THRESHOLD){
                    std::cout << "\033[33m" << "step: " << i << ", " << "prob: " << collision_probability << "\033[0m" << std::endl;
                    std::cout << (affine * trajectory[0]).segment(0, 2).transpose() << " -> " << (affine * trajectory[i]).segment(0, 2).transpose() << std::endl;
                    return true;
                }
            }
        }
    }
    return false;
}

bool SLPDOA::check_collision(const nav_msgs::OccupancyGrid& local_costmap, const std::vector<Eigen::Vector3d>& trajectory, double range)
{
    /*
     * if given trajectory is considered to collide with an obstacle, return true
     */
    Eigen::Affine3d affine;
    try{
        tf::StampedTransform stamped_transform;
        listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), stamped_transform);
        Eigen::Translation<double, 3> trans(stamped_transform.getOrigin().x(), stamped_transform.getOrigin().y(), stamped_transform.getOrigin().z());
        Eigen::Quaterniond q(stamped_transform.getRotation().w(), stamped_transform.getRotation().x(), stamped_transform.getRotation().y(), stamped_transform.getRotation().z());
        affine = trans * q;
    }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
        return true;
    }
    double resolution = local_costmap.info.resolution;
    int size = trajectory.size();
    // for(int i=0;i<size;i++){
    for(int i=0;i<size;i++){
        int xi = round((trajectory[i](0) - local_costmap.info.origin.position.x) / resolution);
        int yi = round((trajectory[i](1) - local_costmap.info.origin.position.y) / resolution);
        //std::cout << xi << ", " << yi << std::endl;
        if(local_costmap.data[xi + local_costmap.info.width * yi] != 0){
            if(trajectory[i].norm() > range){
                return false;
            }else{
                return true;
            }
        }else{
            if(i < PREDICTION_STEP){
                double no_collision_probability = 1;
                for(const auto& obs : obstacle_states_list){
                    no_collision_probability *= 1 - obs.calculate_probability((affine * trajectory[i]).segment(0, 2), i);
                }
                double collision_probability = 1 - no_collision_probability;
                if(collision_probability > COLLISION_PROBABILITY_THRESHOLD){
                    // std::cout << "\033[33m" << "step: " << i << ", " << "prob: " << collision_probability << "\033[0m" << std::endl;
                    // std::cout << (affine * trajectory[i]).segment(0, 2) << std::endl;
                    return true;
                }
            }
        }
    }
    return false;
}

void SLPDOA::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        bool goal_transformed = false;
        geometry_msgs::PoseStamped local_goal_base_link;
        if(local_goal_subscribed){
            try{
                listener.transformPose("/base_link", ros::Time(0), local_goal, local_goal.header.frame_id, local_goal_base_link);
                goal_transformed = true;
            }catch(tf::TransformException ex){
                std::cout << ex.what() << std::endl;
            }
        }
        if(local_goal_subscribed && local_map_updated && odom_updated && goal_transformed){
            std::cout << "=== slp_doa ===" << std::endl;
            double start = ros::Time::now().toSec();
            std::cout << "local goal: \n" << local_goal_base_link << std::endl;
            std::cout << "current_velocity: \n" << current_velocity << std::endl;
            Eigen::Vector3d goal(local_goal_base_link.pose.position.x, local_goal_base_link.pose.position.y, tf::getYaw(local_goal_base_link.pose.orientation));
            std::vector<Eigen::Vector3d> states;
            double target_velocity = get_target_velocity(goal);
            generate_biased_polar_states(N_S, goal, sampling_params, target_velocity, states);
            std::vector<MotionModelDiffDrive::Trajectory> trajectories;
            bool generated = generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z, target_velocity, trajectories);
            if(generated){
                visualize_trajectories(trajectories, 0, 1, 0, N_P * N_H, candidate_trajectories_pub);

                std::cout << "check candidate trajectories" << std::endl;
                std::vector<MotionModelDiffDrive::Trajectory> candidate_trajectories;
                for(const auto& trajectory : trajectories){
                    if(!check_collision(local_map, trajectory.trajectory)){
                        candidate_trajectories.push_back(trajectory);
                    }
                }
                std::cout << "trajectories: " << trajectories.size() << std::endl;
                std::cout << "candidate_trajectories: " << candidate_trajectories.size() << std::endl;
                if(candidate_trajectories.empty()){
                    // if no candidate trajectories
                    // collision checking with relaxed restrictions
                    for(const auto& trajectory : trajectories){
                        if(!check_collision(local_map, trajectory.trajectory, IGNORABLE_OBSTACLE_RANGE)){
                            candidate_trajectories.push_back(trajectory);
                        }
                    }
                    std::cout << "candidate_trajectories(ignore far obstacles): " << candidate_trajectories.size() << std::endl;
                }
                // std::cout << "candidate time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
                if(candidate_trajectories.size() > 0){
                    visualize_trajectories(candidate_trajectories, 0, 0.5, 1, N_P * N_H, candidate_trajectories_no_collision_pub);

                    std::cout << "pickup a optimal trajectory from candidate trajectories" << std::endl;
                    MotionModelDiffDrive::Trajectory trajectory;
                    pickup_trajectory(candidate_trajectories, goal, trajectory);
                    visualize_trajectory(trajectory, 1, 0, 0, selected_trajectory_pub);
                    std::cout << "pickup time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

                    std::cout << "publish velocity" << std::endl;
                    geometry_msgs::Twist cmd_vel;
                    double calculation_time = ros::Time::now().toSec() - start;
                    int delayed_control_index = std::ceil(calculation_time * HZ);
                    std::cout << calculation_time << ", " << delayed_control_index << std::endl;
                    cmd_vel.linear.x = trajectory.velocities[delayed_control_index];
                    cmd_vel.angular.z = trajectory.angular_velocities[delayed_control_index];
                    velocity_pub.publish(cmd_vel);
                    std::cout << "published velocity: \n" << cmd_vel << std::endl;

                    local_map_updated = false;
                    odom_updated = false;
                }else{
                    std::cout << "\033[91mERROR: stacking\033[00m" << std::endl;
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0;
                    cmd_vel.angular.z = 0;
                    velocity_pub.publish(cmd_vel);
                    // for clear
                    std::vector<MotionModelDiffDrive::Trajectory> clear_trajectories;
                    visualize_trajectories(clear_trajectories, 0, 1, 0, N_P * N_H, candidate_trajectories_pub);
                    visualize_trajectories(clear_trajectories, 0, 0.5, 1, N_P * N_H, candidate_trajectories_no_collision_pub);
                    visualize_trajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub);
                }
            }else{
                std::cout << "\033[91mERROR: no optimized trajectory was generated\033[00m" << std::endl;
                std::cout << "\033[91mturn for local goal\033[00m" << std::endl;
                double relative_direction = atan2(local_goal_base_link.pose.position.y, local_goal_base_link.pose.position.x);
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z =  0.2 * ((relative_direction > 0) ? 1 : -1);
                velocity_pub.publish(cmd_vel);
                // for clear
                std::vector<MotionModelDiffDrive::Trajectory> clear_trajectories;
                visualize_trajectories(clear_trajectories, 0, 1, 0, N_P * N_H, candidate_trajectories_pub);
                visualize_trajectories(clear_trajectories, 0, 0.5, 1, N_P * N_H, candidate_trajectories_no_collision_pub);
                visualize_trajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub);
            }
            std::cout << "final time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
        }else{
            if(!local_goal_subscribed){
                std::cout << "waiting for local goal" << std::endl;
            }
            if(!local_map_updated){
                std::cout << "waiting for local map" << std::endl;
            }
            if(!odom_updated){
                std::cout << "waiting for odom" << std::endl;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

