#!/bin/bash

PX4FIRMDIR=$1
CWD=$(dirname "$(readlink -f "$0")")

if [[ "$1" == "" ]]; then
  echo "USAGE: ./install.sh /path/to/PX4-Autopilot_root"
  exit
fi

echo "Installing models.."
cp -f -r $CWD/models/* $PX4FIRMDIR/Tools/simulation/gazebo/sitl_gazebo/models/ -v

echo

echo "Installing airframes.."
cp -f -r $CWD/airframes/* $PX4FIRMDIR/ROMFS/px4fmu_common/init.d-posix/airframes/ -v

echo

echo "Installing worlds.."
cp -f -r $CWD/worlds/* $PX4FIRMDIR/Tools/simulation/gazebo/sitl_gazebo/worlds/ -v

echo

echo "Installing CMake files.."
cp -f $CWD/cmake/sitl_targets_gazebo.cmake $PX4FIRMDIR/src/modules/simulation/simulator_mavlink/ -v

echo

echo "Done"
