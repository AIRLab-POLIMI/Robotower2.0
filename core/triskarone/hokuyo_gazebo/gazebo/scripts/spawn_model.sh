#!/bin/bash

rosrun gazebo_ros spawn_model -file $(rospack find hokuyo-gazebo)/gazebo/hokuyo/model.sdf -sdf \
							  -model hokuyo1 \
                              -x 0.0 -y 4.0 -z 0.019 \
