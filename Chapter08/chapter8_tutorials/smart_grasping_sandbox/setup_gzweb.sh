#!/bin/bash
nohup Xvfb :1 -screen 0 1024x768x16 &> xvfb.log &
DISPLAY=:1.0
export DISPLAY

cd /root
hg clone -r gzweb_1.3.0 https://bitbucket.org/osrf/gzweb
source "/workspace/devel/setup.bash"
source "/usr/share/gazebo/setup.sh"
cd /root
wget -l 2 -nc -r "http://models.gazebosim.org/" --accept gz
cd "models.gazebosim.org"
for i in * ; do tar -zvxf "$i/model.tar.gz" ; done
mkdir -p $HOME/.gazebo/models
cp -vfR * "$HOME/.gazebo/models/"
cd /root/gzweb
GAZEBO_MODEL_PATH=/workspace/src:/workspace/src/universal_robot:~/.gazebo/models:${GAZEBO_MODEL_PATH} ./deploy.sh -m
