#!/bin/bash
set -e

if [ -z "$1" ]; then
  echo "No name of the workspace was supplied"
  exit 1 # terminate and indicate error
else
  NAME_WS=$1
fi


if [ -z "$2" ]; then
  DIR_WS=$HOME
else
  DIR_WS=$2
fi

cd $DIR_WS
if [ ! -d $NAME_WS ]; then
   mkdir $NAME_WS
   cd $NAME_WS
   mkdir build install src
   echo '# setup environment' > setup.bash
   echo 'export LD_LIBRARY_PATH='"$PWD/install/lib"':$LD_LIBRARY_PATH'>> setup.bash
   echo 'export CMAKE_PREFIX_PATH='"$PWD/install"':$CMAKE_PREFIX_PATH'>> setup.bash
   echo 'export PATH='"$PWD/install/bin"':$PATH'>> setup.bash
   echo 'export PKG_CONFIG_PATH='"$PWD/install/lib/pkgconfig"':$PKG_CONFIG_PATH'>> setup.bash  
else
   echo "workspace already exist!"
fi

