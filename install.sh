#!/bin/bash

PX4FIRMDIR=$1
CWD=$(dirname "$(readlink -f "$0")")

if [[ "$1" == "" ]]; then
  echo "USAGE: ./install.sh /path/to/PX4-Autopilot_root"
  exit
fi

echo "Installing CMake files.."
cp -f $CWD/cmake/sitl_targets_gazebo.cmake $PX4FIRMDIR/src/modules/simulation/simulator_mavlink/ -v

echo "installing models.."
cp -f -r $CWD/models/* $PX4FIRMDIR/Tools/simulation/gazebo/sitl_gazebo/models/ -v

echo "installing worlds.."
cp -f -r $CWD/worlds/* $PX4FIRMDIR/Tools/simulation/gazebo/sitl_gazebo/worlds/ -v
#cp -f -r $CWD/hca_models_and_worlds/new_iris/* $PX4FIRMDIR/Tools/simulation/gazebo/sitl_gazebo/models/iris/ -v

echo "Done"
