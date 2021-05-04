echo "Building ROS nodes"
source /opt/ros/melodic/setup.bash
source /home/ran/catkin_ws/devel/setup.bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$PWD/Examples/ROS
alias python=python2
cd Examples/ROS/ORB_SLAM3
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j
