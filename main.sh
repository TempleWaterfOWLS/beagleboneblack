#!/bin/bash

if [ $1 == "kill" ]; then
    echo "Ending Processes..."
    killall python
else
    PROJ_PATH=$PWD
    cd $PROJ_PATH/asm
    python task_asm.py > /dev/null &
    python task_checker.py > /dev/null &
    cd $PROJ_PATH/src
    python basic_control_node.py & 
fi