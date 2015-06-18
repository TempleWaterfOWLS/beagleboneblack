#!/bin/bash
# Script to easily publish ROS info
# Usage: ./publish .1 .1 ---> publishes 0.1 0.1 to topic
# ./publish kill ---> stops all publishing
#
# Define some defaults
RATE='10'

if [ $# -eq 2 ]
    then
    echo "Publishing motor power at rate $RATE with powers $1 $2"
    rostopic pub -r $RATE motor_power beagleboneblack/MotorPower $1 $2 &
fi

if [ $# -eq 1 ] 
    then
    if [ "$1"="kill" ]
	then
	killall rostopic
	fi
fi
