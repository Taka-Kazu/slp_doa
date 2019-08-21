#!/bin/bash

echo "=== run_docker ==="

python update_repositories.py

docker run -it --rm \
  --volume="${PWD}:/root/catkin_ws/src/slp_doa" \
  --net='host' \
  --name="slp_doa_container" \
  slp_doa_docker \
  bash
