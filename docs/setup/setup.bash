# setup environment
export LD_LIBRARY_PATH=/home/embedded/dls_ws/install/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=/home/embedded/dls_ws/install:$CMAKE_PREFIX_PATH
export PATH=/home/embedded/dls_ws/install/bin:$PATH
export PKG_CONFIG_PATH=/home/embedded/dls_ws/install/lib/pkgconfig:$PKG_CONFIG_PATH




# setup environment
export LD_LIBRARY_PATH=/home/embedded/src/xbot2_ws/install/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=/home/embedded/src/xbot2_ws/install:$CMAKE_PREFIX_PATH
export PATH=/home/embedded/src/xbot2_ws/install/bin:$PATH
export GAZEBO_PLUGIN_PATH=/home/embedded/src/xbot2_ws/install/lib:$GAZEBO_PLUGIN_PATH
export PYTHONPATH=/home/embedded/src/xbot2_ws/install/lib/python2.7/dist-packages:/home/embedded/src/xbot2_ws/install/lib/python3/dist-packages:$PYTHONPATH
export ROS_PACKAGE_PATH=/home/embedded/src/xbot2_ws/ros_src:/home/embedded/src/xbot2_ws/install/share:/home/embedded/src/xbot2_ws/install/lib:$ROS_PACKAGE_PATH
export PKG_CONFIG_PATH=/home/embedded/src/xbot2_ws/install/lib/pkgconfig:$PKG_CONFIG_PATH
export HHCM_FOREST_PATH=/home/embedded/src/xbot2_ws:$HHCM_FOREST_PATH

# source env hooks
if [ -d /home/embedded/src/xbot2_ws/install/share/forest_env_hook ]; then
    for f in /home/embedded/src/xbot2_ws/install/share/forest_env_hook/*; do source $f; done
fi
