#!/bin/bash
set -e

nohup Xvfb :1 -screen 0 1024x768x16 &> xvfb.log &
DISPLAY=:1.0
export DISPLAY

source "/workspace/devel/setup.bash"
source "/usr/share/gazebo/setup.sh"

roslaunch smart_grasping_sandbox smart_grasping_sandbox.launch gui:=false gzweb:=true verbose:=true &

sleep 5

cd ~/gzweb
GAZEBO_MODEL_PATH=/workspace/src:/workspace/src/universal_robot:~/.gazebo/models:${GAZEBO_MODEL_PATH} ./start_gzweb.sh &

cd ~/c9sdk
node server.js --listen 0.0.0.0 --port 8181 -w /workspace/src &

cd /workspace/src/smart_grasping_sandbox/notebooks
jupyter notebook --ip=0.0.0.0 --allow-root

