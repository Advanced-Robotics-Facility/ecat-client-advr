#!/bin/bash

actual_dir=$(pwd)

cd /usr/local/MATLAB/R2022a/toolbox/ros/mlroscpp/+ros/+internal

if [ -e CMakeList.txt.tmpl_old ]
then
    mv CMakeList.txt.tmpl_old CMakeList.txt.tmpl
    echo "File: CMakeList.txt.tmpl restored."
else
    echo "File: CMakeList.txt.tmpl already restored."
fi
     
cd /usr/local/MATLAB/R2022a/toolbox/ros/codertarget/+ros/+codertarget/+internal

if [ -e RemoteLnxSystemExecutor_old.m ]
then
    mv RemoteLnxSystemExecutor_old.m  RemoteLnxSystemExecutor.m
    echo "File: RemoteLnxSystemExecutor.m restored."
else
    echo "File: RemoteLnxSystemExecutor.m already restored."
fi    

cd /usr/local/MATLAB/R2022a/toolbox/ros/codertarget/src

if [ -e deploy_build_ros_model_old.sh ]
then
    mv deploy_build_ros_model_old.sh  deploy_build_ros_model.sh
    echo "File: deploy_build_ros_model.sh restored."
else
    echo "File: deploy_build_ros_model.sh already restored."
    
fi
