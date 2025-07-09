#!/bin/bash
set -e 

echo "Starting entrypoint..."
source /opt/ros/overlay_ws/devel/setup.bash
source ${HOME}/workspace/ros/devel/setup.bash
roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=${HOME}/work/binder/rviz_configs/pr2_config.json &
sleep 5 

#export KNOWROB_MONGODB_URI=${MONGODB_URL}/?appname=knowrob
roslaunch --wait knowrob_ros knowrob.launch &

jupyter lab workspaces import ${HOME}/work/binder/BA_project.jupyterlab-workspace

exec "$@"
