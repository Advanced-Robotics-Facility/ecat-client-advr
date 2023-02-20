#!/bin/sh
hw_requested=pos
hw_type=$hw_requested
while true
do
    ec_sim -h $hw_type &  
    pid=$(pgrep ec_sim)
    sleep 3
    echo KILL EC SIM
    kill -2 $pid
    echo $hw_type $hw_requested
    wait $pid
    echo $hw_type $hw_requested
    if [ "$hw_type" = "idle" ];then
        hw_type=$hw_requested
    else
        hw_type=idle
    fi
done

