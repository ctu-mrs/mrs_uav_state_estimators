#!/bin/bash

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"

PACKAGE_PATH=$( rospack find mrs_uav_state_estimation )

cp $PACKAGE_PATH/plot_juggler/manager_debug.xml /tmp/manager_debug.xml

sed -i "s/uav[0-9]/$UAV_NAME/g" /tmp/manager_debug.xml
