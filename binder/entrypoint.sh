#!/bin/bash
set -ex

echo "Starting entrypoint..."

source /opt/ros/noetic/setup.bash
source /opt/ros/overlay_ws/devel/setup.bash

echo "Starting roscore..."
roscore &
sleep 5

echo "Jupyter version:"
jupyter --version

#roslaunch --wait knowrob_ros knowrob.launch &
#sleep 5

echo "Starting Jupyter..."
#jupyter lab workspaces import ${HOME}/work/binder/BA_workspace.jupyterlab-workspace

#jupyter lab --ip=0.0.0.0 --no-browser --allow-root
#jupyter lab  --ip=0.0.0.0 --no-browser --NotebookApp.notebook_dir='/home/jovyan/work/binder/BA_workspace.jupyterlab-workspace'
#jupyter lab workspaces import ${HOME}/work/binder/BA_workspace.jupyterlab-workspace
#echo "Launching Jupyter in $(pwd)..."

exec jupyter lab \
    --debug \
    --ip=0.0.0.0 \
    --no-browser \
    --ServerApp.token='' \
    --ServerApp.password=''
#"$@"
