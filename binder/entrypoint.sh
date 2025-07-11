#!/bin/bash
set -e 

echo "Starting entrypoint..."

source /opt/ros/noetic/setup.bash
source /opt/ros/overlay_ws/devel/setup.bash

roscore &

#export KNOWROB_MONGODB_URI=${MONGODB_URL}/?appname=knowrob
roslaunch --wait knowrob_ros knowrob.launch &
sleep 5

jupyter lab workspaces import ${HOME}/work/binder/BA_project.jupyterlab-workspace

exec "$@"
