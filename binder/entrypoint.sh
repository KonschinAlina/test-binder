#!/bin/bash
set -e 

echo "Starting entrypoint..."

source /opt/ros/noetic/setup.bash
source /opt/ros/overlay_ws/devel/setup.bash

roscore &
sleep 5

roslaunch --wait knowrob_ros knowrob.launch &
sleep 5

jupyter lab workspaces import ${HOME}/work/binder/BA_workspace.jupyterlab-workspace

#jupyter lab --ip=0.0.0.0 --no-browser --allow-root


#jupyter lab  --ip=0.0.0.0 --no-browser --NotebookApp.notebook_dir='/home/jovyan/work/binder/BA_workspace.jupyterlab-workspace'

#jupyter lab workspaces import ${HOME}/work/binder/BA_workspace.jupyterlab-workspace


#echo "Launching Jupyter in $(pwd)..."

exec "$@"
