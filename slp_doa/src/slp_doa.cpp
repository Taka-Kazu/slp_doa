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

void SLPDOA::set_collision_prediction_params(double prediction_time_, double dt_, double threshold_)
{
    PREDICTION_TIME = prediction_time_;
    DT = dt_;
    PREDICTION_STEP = std::round(PREDICTION_TIME / DT);
    COLLISION_PROBABILITY_THRESHOLD = threshold_;
}
