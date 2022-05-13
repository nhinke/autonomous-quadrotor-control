#!/usr/bin/env bash

# actual bash script used to launch simulation found at /aqc_driver/scripts/launch-sim.sh...
# that script uses locate to find where QGroundControl and PX4-Autopilot are installed...
# if you find that it takes too long to launch the simulation as a result, please define the actual path to your installations within that script...

DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PATH_TO_SIM_SCRIPT_FROM_DIR="../aqc_driver/scripts/"
SIM_SCRIPT_NAME="launch-sim.sh"

cd $DIR/$PATH_TO_SIM_SCRIPT_FROM_DIR
./$SIM_SCRIPT_NAME
