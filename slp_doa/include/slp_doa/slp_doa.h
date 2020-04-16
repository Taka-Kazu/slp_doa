#ifndef __SLP_DOA_H
#define __SLP_DOA_H

#include "state_lattice_planner/state_lattice_planner.h"
#include "dynamic_obstacle_avoidance_planner/obstacle_tracker_kf.h"

#include <omp.h>

class SLPDOA: public StateLatticePlanner
{
public:
    SLPDOA(void);

    class ObstacleStates
    {
    public:
        ObstacleStates(void);
        ObstacleStates(int, double);

        double calculate_probability(const Eigen::Vector2d&, int) const;
        double square(double) const;

        std::vector<Eigen::Vector2d> pos;
        std::vector<Eigen::Vector2d> vel;
        std::vector<Eigen::Matrix2d> vcov;
        double resolution;
    };

    class ProbabilityWithTimeStep
    {
    public:
        ProbabilityWithTimeStep(void);
        ProbabilityWithTimeStep(unsigned int, double);

        unsigned int time_step;
        double probability;
    };

    template<typename TYPE>
    ProbabilityWithTimeStep get_collision_probability(const state_lattice_planner::ObstacleMap<TYPE>&, const std::vector<Eigen::Vector3d>&, const Eigen::Affine3d&, std::vector<double>&);
    void set_collision_prediction_params(double, double, double);

    ObstacleTrackerKF tracker;
    std::vector<ObstacleStates> obstacle_states_list;
private:

    double PREDICTION_TIME;
    unsigned int PREDICTION_STEP;
    double DT;
    double COLLISION_PROBABILITY_THRESHOLD;
};

template<typename TYPE>
SLPDOA::ProbabilityWithTimeStep SLPDOA::get_collision_probability(const state_lattice_planner::ObstacleMap<TYPE>& local_costmap, const std::vector<Eigen::Vector3d>& trajectory, const Eigen::Affine3d& affine, std::vector<double>& probabilities)
{
    /*
     * if given trajectory is considered to collide with an obstacle, return collision time step
     * if not collision, this function returns boost::none
     */
    const unsigned int map_size = local_costmap.data.size();
    const unsigned int size = trajectory.size();
    double max_collision_probability = 0;
    unsigned int max_collision_probability_time = 0;
    probabilities.resize(trajectory.size());
    for(unsigned int i=1;i<size;i++){
        // std::cout << "i: " << i << std::endl;
        probabilities[i] = 0.0;
        unsigned int index = local_costmap.get_index_from_xy(trajectory[i](0), trajectory[i](1));
        if(index < 0 || map_size <= index){
            // std::cout << "invalid index" << std::endl;
            continue;
        }
        Eigen::Vector3d transformed_trajectory_position = affine * trajectory[i];
        if(local_costmap.data[index] != 0){
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

#endif// __SLP_DOA_H
