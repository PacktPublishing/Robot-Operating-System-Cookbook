#!/bin/bash
set -e

nohup Xvfb :1 -screen 0 1024x768x16 &> xvfb.log &
DISPLAY=:1.0
export DISPLAY

source "/workspace/devel/setup.bash"
source "/usr/share/gazebo/setup.sh"

roslaunch smart_grasping_sandbox smart_grasping_sandbox.launch gui:=false gzweb:=true verbose:=true &

sleep 20

python /workspace/src/smart_grasping_sandbox/nodes/grasp_quality.py $1 $2
