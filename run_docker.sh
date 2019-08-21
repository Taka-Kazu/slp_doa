#!/bin/bash

echo "=== run_docker ==="

python update_repositories.py

docker run -it --rm \
  --volume="${PWD}/slp_doa:/root/catkin_ws/src/slp_doa" \
  --volume="${PWD}/dynamic_obstacle_avoidance_planner:/root/catkin_ws/src/dynamic_obstacle_avoidance_planner" \
  --volume="${PWD}/state_lattice_planner:/root/catkin_ws/src/state_lattice_planner" \
  --net='host' \
  --name="slp_doa_container" \
  slp_doa_docker \
  bash
