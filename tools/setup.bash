# Sourcing all catkin workspaces
source ~/sunfest_2019/src/catkin_ws/devel/setup.bash
source ~/sunfest_2019/src/calibration/kalibr_workspace/devel/setup.bash

# making executable
chmod u+x ~/sunfest_2019/tools/davis
export PATH=$PATH:~/sunfest_2019/tools

