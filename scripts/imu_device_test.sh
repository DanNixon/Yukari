#!/bin/sh

IMU_APP=./IMUGrabberTest
DATE=`date +"%m_%y_%d_%H_%M_%S"`

$IMU_APP --grabber "attitude(port=/dev/ttyACM0)" --datafile "$DATE"_attitude.csv &
$IMU_APP --grabber "teensy(port=/dev/ttyACM1)" --datafile "$DATE"_teensy.csv &
$IMU_APP --grabber "stm32(port=/dev/ttyUSB0,position=false)" --datafile "$DATE"_stm32.csv &

