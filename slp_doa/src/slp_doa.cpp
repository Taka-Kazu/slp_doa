#include "slp_doa/slp_doa.h"

SLPDOA::SLPDOA(void)
{
    PREDICTION_TIME = 3.5;
    COLLISION_PROBABILITY_THRESHOLD = 0.1;
    PREDICTION_STEP = PREDICTION_TIME / HZ;
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

double SLPDOA::ObstacleStates::square(double val) const
{
    return val * val;
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
    double avoidance_radius = 0.8;
    double probability = 0;
    for(double y=position(1)-avoidance_radius;y<position(1)+avoidance_radius;y+=resolution){
        double x0 = position(0) - sqrt(square(avoidance_radius) - square(y - position(1)));
        double x1 = position(0) + sqrt(square(avoidance_radius) - square(y - position(1)));
        for(double x=x0;x<x1;x+=resolution){
            Eigen::Vector2d p(x, y);
            double probability_ =  coeff * std::exp(-0.5 * (p - mu).transpose() * sigma.inverse() * (p - mu));
            probability += probability_;
        }
    }
    probability *= square(resolution);
    // if((position - mu).norm() < avoidance_radius){
    //     std::cout << "t=" << step << std::endl;
    //     std::cout << "position:\n" << position.transpose() << std::endl;
    //     std::cout << "mu:\n" << mu.transpose() << std::endl;
    //     std::cout << "sigma:\n" << sigma << std::endl;
    //     std::cout << "prob: " << probability << std::endl;
    // }
    if(std::isnan(probability) or std::isinf(probability)){
        std::cout << probability << std::endl;
        exit(-1);
    }
    return probability;
}

SLPDOA::ProbabilityWithTimeStep::ProbabilityWithTimeStep(void)
:time_step(0), probability(0.0)
{
}

SLPDOA::ProbabilityWithTimeStep::ProbabilityWithTimeStep(unsigned int t_, double p_)
:time_step(t_), probability(p_)
{
}

template<typename TYPE>
SLPDOA::ProbabilityWithTimeStep SLPDOA::get_collision_probability(const state_lattice_planner::ObstacleMap<TYPE>& local_costmap, const std::vector<Eigen::Vector3d>& trajectory, const Eigen::Affine3d& affine, std::vector<double>& probabilities)
{
    /*
     * if given trajectory is considered to collide with an obstacle, return collision time step
     * if not collision, this function returns boost::none
     */
    double resolution = local_costmap.info.resolution;
    const unsigned int map_size = local_costmap.data.size();
    const unsigned int size = trajectory.size();
    double max_collision_probability = 0;
    unsigned int max_collision_probability_time = 0;
    probabilities.resize(trajectory.size());
    for(unsigned int i=1;i<size;i++){
        // std::cout << "i: " << i << std::endl;
        probabilities[i] = 0.0;
        int xi = std::round((trajectory[i](0) - local_costmap.origin_x) / resolution);
        int yi = std::round((trajectory[i](1) - local_costmap.origin_y) / resolution);
        unsigned int index = xi + local_costmap.width * yi;
        if(index < 0 || map_size <= index){
            // std::cout << "invalid index" << std::endl;
            continue;
        }
        Eigen::Vector3d transformed_trajectory_position = affine * trajectory[i];
        if(local_costmap.data[xi + local_costmap.width * yi] != 0){
            // return ProbabilityWithTimeStep(i, 1.0);
            probabilities[i] = 1.0;
            max_collision_probability = 1.0;
            max_collision_probability_time = i;
        }else{
            if(i < PREDICTION_STEP){
                double no_collision_probability = 1;
                int count = 0;
                for(const auto& obs : obstacle_states_list){
                    Eigen::Vector2d vector_from_obs_to_traj = transformed_trajectory_position .segment(0, 2) - obs.pos[i];
                    double distance_from_trajectory = vector_from_obs_to_traj.norm();
                    if((distance_from_trajectory > 3)
                       || (distance_from_trajectory > 5 && obs.vel[i].dot(vector_from_obs_to_traj) < 0.0)){
                        // obs is far from traj or
                        // is not coming to traj
                        continue;
                    }
                    no_collision_probability *= (1 - obs.calculate_probability(transformed_trajectory_position.segment(0, 2), i));
                    count++;
                }
                double collision_probability = 1 - no_collision_probability;
                // std::cout << "i: " << i << ", " << "collision probability: " << collision_probability << std::endl;
                if(collision_probability > max_collision_probability){
                    max_collision_probability = collision_probability;
                    max_collision_probability_time = i;
                }
                probabilities[i] = collision_probability;
            }else{
                // std::cout << "i: " << i << ", out of prediction time" << std::endl;
            }
        }
    }
    return ProbabilityWithTimeStep(max_collision_probability_time, max_collision_probability);
}

void SLPDOA::set_collision_prediction_params(double prediction_time_, double dt_, double threshold_)
{
    PREDICTION_TIME = prediction_time_;
    DT = dt_;
    PREDICTION_STEP = std::round(PREDICTION_TIME / DT);
    COLLISION_PROBABILITY_THRESHOLD = threshold_;
}
