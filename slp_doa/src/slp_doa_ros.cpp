#include "slp_doa/slp_doa_ros.h"

SLPDOAROS::SLPDOAROS(void)
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

    slp_doa_planner.set_collision_prediction_params(PREDICTION_TIME, DT, COLLISION_PROBABILITY_THRESHOLD);

    obstacles_predicted_path_pub = local_nh.advertise<geometry_msgs::PoseArray>("/obstacle_predicted_paths", 1);
    probability_map_pub = local_nh.advertise<nav_msgs::OccupancyGrid>("/probability_map", 1);
    obstacle_pose_sub = nh.subscribe("/dynamic_obstacles", 1, &SLPDOAROS::obstacle_pose_callback, this);
    std::cout << std::endl;
}

void SLPDOAROS::obstacle_pose_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
    std::cout << "obstacle pose callback" << std::endl;
    geometry_msgs::PoseArray obstacle_pose = *msg;
    try{
        for(auto& p : obstacle_pose.poses){
            geometry_msgs::PoseStamped p_;
            p_.header = obstacle_pose.header;
            p_.pose = p;
            listener.waitForTransform(WORLD_FRAME, p_.header.frame_id, p_.header.stamp, ros::Duration(1.0));
            listener.transformPose(WORLD_FRAME, p_, p_);
            p = p_.pose;
        }
        obstacle_pose.header.frame_id = WORLD_FRAME;
    }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
        return;
    }

    slp_doa_planner.tracker.set_obstacles_pose(obstacle_pose);
    int obs_num = slp_doa_planner.tracker.obstacles.size();

    if(local_map.data.empty()){
        return;
    }
    // prediction
    slp_doa_planner.obstacle_states_list.clear();
    slp_doa_planner.obstacle_states_list.reserve(obs_num);
    for(auto it=slp_doa_planner.tracker.obstacles.begin();it!=slp_doa_planner.tracker.obstacles.end();++it){
        SLPDOA::ObstacleStates obstacle_states(PREDICTION_STEP, local_map.info.resolution);
        Obstacle obs = it->second;
        for(unsigned int i=0;i<PREDICTION_STEP;i++){
            obstacle_states.pos.push_back(obs.x.segment(0, 2));
            obstacle_states.vel.push_back(obs.x.segment(2, 2));
            obstacle_states.vcov.push_back(obs.p.block(0, 0, 2, 2));
            obs.predict(DT);
        }
        slp_doa_planner.obstacle_states_list.push_back(obstacle_states);
    }

    // for debug
    geometry_msgs::PoseArray obstacle_paths;
    obstacle_paths.header = obstacle_pose.header;
    obstacle_paths.header.frame_id = WORLD_FRAME;
    obstacle_paths.poses.clear();
    for(int i=0;i<obs_num;i++){
        // std::cout << "obstacle " << i << ": "<< std::endl;
        // std::cout << slp_doa_planner.obstacle_states_list[i].pos[0] << std::endl;
        // std::cout << slp_doa_planner.obstacle_states_list[i].vel[0] << std::endl;
        for(unsigned int j=0;j<PREDICTION_STEP;j++){
            geometry_msgs::Pose p;
            p.position.x = slp_doa_planner.obstacle_states_list[i].pos[j](0);
            p.position.y = slp_doa_planner.obstacle_states_list[i].pos[j](1);
            p.position.z = 0.0;
            p.orientation = tf::createQuaternionMsgFromYaw(atan2(slp_doa_planner.obstacle_states_list[i].vel[j](1), slp_doa_planner.obstacle_states_list[i].vel[j](0)));
            // std::cout << p << std::endl;
            obstacle_paths.poses.push_back(p);
        }
    }
    // std::cout << "obs num: " << obs_num << std::endl;
    std::cout << "obs path num: " << obstacle_paths.poses.size() << std::endl;
    obstacles_predicted_path_pub.publish(obstacle_paths);
}

SLPDOA::ProbabilityWithTimeStep SLPDOAROS::get_collision_probability(const nav_msgs::OccupancyGrid& local_costmap, const std::vector<Eigen::Vector3d>& trajectory, std::vector<double>& probabilities)
{
    // std::cout << "get collision probability" << std::endl;
    Eigen::Affine3d affine;
    try{
        tf::StampedTransform stamped_transform;
        // listener.waitForTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time::now(), ros::Duration(1.0));
        listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), stamped_transform);
        Eigen::Translation<double, 3> trans(stamped_transform.getOrigin().x(), stamped_transform.getOrigin().y(), stamped_transform.getOrigin().z());
        Eigen::Quaterniond q(stamped_transform.getRotation().w(), stamped_transform.getRotation().x(), stamped_transform.getRotation().y(), stamped_transform.getRotation().z());
        affine = trans * q;
    }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
        return ProbabilityWithTimeStep(0, 1.0);
    }
    state_lattice_planner::ObstacleMap<int> local_costmap_;
    ////
    ////
    return slp_doa_planner.get_collision_probability(local_costmap_, trajectory, affine, probabilities);
}

void SLPDOAROS::visualize_trajectories_with_probability(const std::vector<MotionModelDiffDrive::Trajectory>& trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher& pub, const std::vector<std::vector<double> >& probabilities, const std::vector<unsigned int>& candidate_indices)
{
    visualization_msgs::MarkerArray v_trajectories;
    int count = 0;
    const int size = trajectories.size();
    for(;count<size;count++){
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = ROBOT_FRAME;
        v_trajectory.header.stamp = ros::Time::now();
        // v_trajectory.color.r = r;
        // v_trajectory.color.g = g;
        // v_trajectory.color.b = b;
        v_trajectory.color.a = 0.8;
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::ADD;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectory.pose.orientation.w = 1.0;
        v_trajectory.scale.x = 0.05;
        geometry_msgs::Point p;
        std_msgs::ColorRGBA color;
        // for(const auto& pose : trajectories[count].trajectory){
        //     p.x = pose(0);
        //     p.y = pose(1);
        for(unsigned int i=0;i<trajectories[count].trajectory.size();i++){
            p.x = trajectories[count].trajectory[i](0);
            p.y = trajectories[count].trajectory[i](1);
            v_trajectory.points.push_back(p);
            if(std::find(candidate_indices.begin(), candidate_indices.end(), count) != candidate_indices.end()){
                color.r = r * (1 - probabilities[count][i]);
                color.g = g * (1 - probabilities[count][i]);
                color.b = b * (1 - probabilities[count][i]);
            }else{
                color.r = 0;
                color.g = 1 * (1 - probabilities[count][i]);
                color.b = 0;
            }
            color.a = 0.8;
            v_trajectory.colors.push_back(color);
        }
        v_trajectories.markers.push_back(v_trajectory);
    }
    for(;count<trajectories_size;){
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = ROBOT_FRAME;
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::DELETE;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectories.markers.push_back(v_trajectory);
        count++;
    }
    pub.publish(v_trajectories);
}

void SLPDOAROS::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        bool goal_transformed = false;
        geometry_msgs::PoseStamped local_goal_base_link;
        if(local_goal_subscribed){
            try{
                listener.waitForTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time::now(), ros::Duration(1.0));
                listener.transformPose(ROBOT_FRAME, ros::Time(0), local_goal, local_goal.header.frame_id, local_goal_base_link);
                goal_transformed = true;
            }catch(tf::TransformException ex){
                std::cout << ex.what() << std::endl;
            }
        }
        if(local_goal_subscribed && local_map_updated && odom_updated && goal_transformed){
            std::cout << "=== slp_doa ===" << std::endl;
            double start = ros::Time::now().toSec();
            static int last_trajectory_num = 0;
            std::cout << "local goal: \n" << local_goal_base_link << std::endl;
            std::cout << "current_velocity: \n" << current_velocity << std::endl;
            Eigen::Vector3d goal(local_goal_base_link.pose.position.x, local_goal_base_link.pose.position.y, tf::getYaw(local_goal_base_link.pose.orientation));
            std::vector<Eigen::Vector3d> states;
            double target_velocity = get_target_velocity(goal);
            slp_doa_planner.generate_biased_polar_states(N_S, goal, sampling_params, target_velocity, states);
            std::vector<MotionModelDiffDrive::Trajectory> trajectories;
            bool generated = slp_doa_planner.generate_trajectories(states, current_velocity.linear.x, current_velocity.angular.z, target_velocity, trajectories);
            bool turn_flag = false;
            double relative_direction = atan2(local_goal_base_link.pose.position.y, local_goal_base_link.pose.position.x);
            if(goal.segment(0, 2).norm() < 0.1){
                generated = false;
            }else if(fabs(relative_direction) > TURN_DIRECTION_THRESHOLD){
                // if(fabs(goal(2)) > TURN_DIRECTION_THRESHOLD){
                    generated = false;
                    turn_flag = true;
                // }
            }
            std::cout << "trajectories generation time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
            if(generated){
                std::cout << "check candidate trajectories" << std::endl;
                const unsigned int generated_trajectories_size = trajectories.size();
                std::cout << "trajectories: " << generated_trajectories_size << std::endl;
                std::vector<MotionModelDiffDrive::Trajectory> candidate_trajectories(generated_trajectories_size);
                std::vector<std::vector<double> > probabilities(generated_trajectories_size);

                #pragma omp parallel for
                for(unsigned int i=0;i<generated_trajectories_size;i++){
                    auto trajectory = trajectories[i];
                    std::vector<double> prob;
                    auto collision_probability = get_collision_probability(local_map, trajectory.trajectory, prob);
                    probabilities[i] = prob;
                    // std::cout << "t, p: " << collision_probability.time_step << ", " << collision_probability.probability << std::endl;;
                    // std::cout << "collision probability time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
                    if(collision_probability.probability < COLLISION_PROBABILITY_THRESHOLD){
                        // no collision
                        candidate_trajectories[i] = trajectory;
                    }
                }

                std::vector<unsigned int> candidate_indices;
                unsigned int no_collision_count = 0;
                for(auto it=candidate_trajectories.begin();it!=candidate_trajectories.end();){
                    if(it->trajectory.size() == 0){
                        it = candidate_trajectories.erase(it);
                    }else{
                        ++it;
                        candidate_indices.emplace_back(no_collision_count);
                    }
                    no_collision_count++;
                }

                visualize_trajectories_with_probability(trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_pub, probabilities, candidate_indices);

                std::cout << "candidate_trajectories: " << candidate_trajectories.size() << std::endl;
                std::cout << "candidate time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
                if(candidate_trajectories.size() > 0){
                    visualize_trajectories(candidate_trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_no_collision_pub);

                    std::cout << "pickup a optimal trajectory from candidate trajectories" << std::endl;
                    MotionModelDiffDrive::Trajectory trajectory;
                    slp_doa_planner.pickup_trajectory(candidate_trajectories, goal, trajectory);
                    visualize_trajectory(trajectory, 1, 0, 0, selected_trajectory_pub);
                    std::cout << "pickup time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

                    std::cout << "publish velocity" << std::endl;
                    geometry_msgs::Twist cmd_vel;
                    double calculation_time = ros::Time::now().toSec() - start;
                    int delayed_control_index = std::ceil(calculation_time * HZ) + 1;
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
                    // visualize_trajectories(clear_trajectories, 0, 1, 0, N_P * N_H, candidate_trajectories_pub);
                    // visualize_trajectories(clear_trajectories, 0, 0.5, 1, N_P * N_H, candidate_trajectories_no_collision_pub);
                    visualize_trajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub);
                }
            }else{
                std::cout << "\033[91mERROR: no optimized trajectory was generated\033[00m" << std::endl;
                std::cout << "\033[91mturn for local goal\033[00m" << std::endl;
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                double yaw_rate = turn_flag ? relative_direction : goal(2);
                double d_yaw_rate =  (yaw_rate - current_velocity.angular.z) / DT;
                d_yaw_rate = std::min(std::max(d_yaw_rate, -MAX_D_YAWRATE), MAX_D_YAWRATE);
                yaw_rate = current_velocity.angular.z + d_yaw_rate * DT;
                cmd_vel.angular.z = std::min(std::max(yaw_rate, -MAX_YAWRATE), MAX_YAWRATE);
                velocity_pub.publish(cmd_vel);
                // for clear
                std::vector<MotionModelDiffDrive::Trajectory> clear_trajectories;
                visualize_trajectories(clear_trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub);
                visualize_trajectories(clear_trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_no_collision_pub);
                visualize_trajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub);
            }
            probability_map_pub.publish(local_map);
            last_trajectory_num = trajectories.size();
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

template<typename TYPE>
void SLPDOAROS::get_obstacle_map(const nav_msgs::OccupancyGrid& input_map, state_lattice_planner::ObstacleMap<TYPE>& output_map)
{
    output_map.set_shape(input_map.info.width, input_map.info.height, input_map.info.resolution);
    output_map.data.clear();
    for(const auto& data : input_map.data){
        output_map.data.emplace_back(data);
    }
}
