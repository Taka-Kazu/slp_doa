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

#endif// __SLP_DOA_H
