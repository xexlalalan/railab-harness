# source this in non-interactive shells on the harness PC
source /opt/ros/lyrical/setup.bash
export PYTHONPATH=$HOME/bunny-harness-dev/ros-venv/lib/python3.14/site-packages:${PYTHONPATH:-}
export ROS_DOMAIN_ID=36
source $HOME/bunny-harness-dev/install/setup.bash
