#include "slp_doa/slp_doa.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "slp_doa");
    SLPDOA slp_doa;
    slp_doa.process();
    return 0;
};
