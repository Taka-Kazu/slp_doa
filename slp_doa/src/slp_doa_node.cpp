#include "slp_doa/slp_doa_ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slp_doa");
    SLPDOAROS planner;
    planner.process();
    return 0;
};
