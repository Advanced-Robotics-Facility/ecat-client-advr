#!/bin/bash

actual_dir=$(pwd)

cd /usr/local/MATLAB/R2020b/sys/os/glnxa64/

if [[ `lsb_release -cs` == "focal" ]]; then 
   if [ -e libstdc++.so.6.bak ]
   then
      echo "File: libstdc++.so.6 already copied into bak file."
   else
      echo "File libstdc++.so.6: Copied actual configuration into back file"
      mv libstdc++.so.6 libstdc++.so.6.bak
      ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6  libstdc++.so.6
   fi   
fi

cd /usr/local/MATLAB/R2020b/toolbox/ros/utilities/+ros/+codertarget/+internal

if [ -e onAfterCodeGen_old.m ]
then
    echo "File: onAfterCodeGen.m already copied into old file."
else
    echo "File onAfterCodeGen.m : Copied actual configuration into old file"
    mv onAfterCodeGen.m  onAfterCodeGen_old.m
fi

cp $actual_dir/onAfterCodeGen.m .
echo "File: onAfterCodeGen.m patched."


cd /usr/local/MATLAB/R2020b/toolbox/ros/codertarget

if [ -e rosdevice_old.m ]
then
    echo "File: rosdevice.m already copied into old file."
else
    echo "File rosdevice.m: Copied actual configuration into old file"
    mv rosdevice.m  rosdevice_old.m
fi

cp $actual_dir/rosdevice.m .
echo "File: rosdevice.m patched."


cd /usr/local/MATLAB/R2020b/toolbox/ros/codertarget/src

if [ -e deploy_build_ros_model_old.sh ]
then
    echo "File: deploy_build_ros.sh already copied into old file."
else
    echo "File deploy_build_ros.sh: Copied actual configuration into old file"
    mv deploy_build_ros_model.sh  deploy_build_ros_model_old.sh
fi

cp $actual_dir/deploy_build_ros_model.sh .
echo "File: deploy_build_ros.sh patched."
