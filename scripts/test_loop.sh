#!/bin/sh

while true
do
    ec_simple_trajectory &
    pid=$!

    wait "$pid"
    status=$?

    if [ "$status" -eq 0 ]; then
        echo "Success!! Restarting..."
    else
        echo "Error: $status"
        exit "$status"
    fi
done
