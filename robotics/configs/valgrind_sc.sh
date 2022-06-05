#!/bin/bash

# Define a variable with path to libsoft_speed.a
PATH_TO_LIB=/workspace/robotics/ros2/src/motion_control/lib

# 
TARGET_BIN=/workspace/robotics/ros2/install/motion_control/lib/motion_control/speed_controller

# Store the current LD_LIBRARY_PATH environment variable
LD_PATH=${LD_LIBRARY_PATH}

# Check if path already in LD_LIBRARY_PATH
if [[ $LD_PATH == *"$PATH_TO_LIB"* ]];
then
    echo "Variable in path. Not adding."
else
    echo "Variable not in path. Adding."
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PATH_TO_LIB
    echo "Added to path"
fi

# Execute valgrind with a ten seconds timeout
timeout 10 valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file='output.txt' -v $TARGET_BIN
