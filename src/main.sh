#!/bin/bash

if [ "$1" == "kill" ]; then
    echo "Ending Processes..."
    killall python
else
    python motor_control_node.py &
    python motor_comm_node.py > /dev/null &
fi
