#ifndef __SLP_DOA_H
#define __SLP_DOA_H

#include <ros/ros.h>

#include "state_lattice_planner/state_lattice_planner.h"
#include "dynamic_obstacle_avoidance_planner/obstacle_tracker_kf.h"

class SLPDOA: public StateLatticePlanner
{
public:
    SLPDOA(void);
private:
};

#endif// __SLP_DOA_H
