#!/bin/bash

# Launch the ROS core and web tools when containter starts
#source ${HOME}/workspace/ros/devel/setup.bash
source /opt/ros/overlay_ws/devel/setup.bash

roscore &
roslaunch --wait rvizweb rvizweb.launch config_file:=${HOME}/work/binder/rviz_configs/pr2_config.json &

# Launch Knowrob
export KNOWROB_MONGODB_URI=${MONGODB_URL}/?appname=knowrob
roslaunch --wait knowrob knowrob.launch &

#cp ${HOME}/test-binder/binder/webapps.json opt/ros/overlay_ws/src/rvizweb/webapps/app.json

# The following line will allow the binderhub start Jupyterlab, should be at the end of the entrypoint.
exec "$@"
