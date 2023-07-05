#!/bin/bash

actual_dir=$(pwd)

cd /usr/local/MATLAB/R2022a/toolbox/ros/mlroscpp/+ros/+internal

if [ -e CMakeList.txt.tmpl_old ]
then
    echo "File: CMakeList.txt.tmpl already copied into old file."
else
    echo "File CMakeList.txt.tmpl : Copied actual configuration into old file"
    mv CMakeList.txt.tmpl  CMakeList.txt.tmpl_old
fi

cp $actual_dir/CMakeList.txt.tmpl .
echo "File: CMakeList.txt.tmpl patched."

cd /usr/local/MATLAB/R2022a/toolbox/ros/codertarget/+ros/+codertarget/+internal

if [ -e RemoteLnxSystemExecutor_old.m ]
then
    echo "File: RemoteLnxSystemExecutor.m already copied into old file."
else
    echo "File RemoteLnxSystemExecutor.m: Copied actual configuration into old file"
    mv RemoteLnxSystemExecutor.m  RemoteLnxSystemExecutor_old.m
fi

cp $actual_dir/RemoteLnxSystemExecutor.m .
echo "File: RemoteLnxSystemExecutor.m patched."


cd /usr/local/MATLAB/R2022a/toolbox/ros/codertarget/src

if [ -e deploy_build_ros_model_old.sh ]
then
    echo "File: deploy_build_ros.sh already copied into old file."
else
    echo "File deploy_build_ros.sh: Copied actual configuration into old file"
    mv deploy_build_ros_model.sh  deploy_build_ros_model_old.sh
fi

cp $actual_dir/deploy_build_ros_model.sh .
echo "File: deploy_build_ros.sh patched."
