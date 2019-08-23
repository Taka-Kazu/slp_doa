#!/bin/bash

set -e

ldconfig

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

exec "$@"
