#!/bin/sh
set -e
while true
do
    ec_hydraulic  &  
    pid=$(pgrep ec_hydraulic)
    wait $pid
    if [ $? -eq 0 ]; then
    	echo "Success!!"
    else
    	echo "Error:" $?
    	exit
    fi
done

