#!/bin/bash

actual_dir=$(pwd)

cd /usr/local/MATLAB/R2020b/sys/os/glnxa64/

if [[ `lsb_release -cs` == "focal" ]]; then 
   if [ -e libstdc++.so.6.bak ]
   then
      rm libstdc++.so.6
      mv libstdc++.so.6.bak libstdc++.so.6
      echo "File: libstdc++.so.6 restored."
   else
      echo "File: libstdc++.so.6 already restored."
   fi   
fi

cd /usr/local/MATLAB/R2020b/toolbox/ros/utilities/+ros/+codertarget/+internal

if [ -e onAfterCodeGen_old.m ]
then
    mv onAfterCodeGen_old.m onAfterCodeGen.m
    echo "File: onAfterCodeGen.m restored."
else
    echo "File: onAfterCodeGen.m already restored."
fi

cd /usr/local/MATLAB/R2020b/toolbox/ros/codertarget

if [ -e rosdevice_old.m ]
then
    mv rosdevice_old.m  rosdevice.m
    echo "File: rosdevice.m restored."
else
    echo "File: rosdevice.m already restored."
fi

cd /usr/local/MATLAB/R2020b/toolbox/ros/codertarget/src

if [ -e deploy_build_ros_model_old.sh ]
then
    mv deploy_build_ros_model_old.sh  deploy_build_ros_model.sh
    echo "File: deploy_build_ros_model_old.sh restored."
else
    echo "File: deploy_build_ros_model_old.sh already restored."
    
fi
