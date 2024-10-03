#!/bin/bash


FILENAME=urdf/box.sdf
rosrun gazebo_ros spawn_model -file $FILENAME -sdf -x 0 -y 0 -z 0 -model 222HOGEBOX