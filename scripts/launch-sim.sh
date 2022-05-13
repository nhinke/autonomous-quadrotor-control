#!/bin/bash

# set simulation model to either IRIS or SOLO
model="SOLO"

# launch QGroundControl in background
qgc="$(locate QGroundControl.AppImage)"
$qgc &

# launch mavros in background with local host fcu_url
roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557" & 

# launch gazebo SITL simulation located at JHU
export PX4_HOME_LAT=39.32647166152788
export PX4_HOME_LON=-76.62155985832216
export PX4_HOME_ALT=68

launch_cmd="gazebo"
if [ "${model,,}" = "iris" ]; then
    echo "MODEL VALUE: using model IRIS in simulation"
    launch_cmd="gazebo"
elif [ "${model,,}" = "solo" ]; then
    echo "MODEL VALUE: using model SOLO in simulation"
    launch_cmd="gazebo_solo"
else
    echo "ERROR: invalid model choice, using default value of IRIS"
    launch_cmd="gazebo"
fi

px4="$(locate locate PX4-Autopilot | grep /PX4-Autopilot$)"
(cd $px4 && make px4_sitl $launch_cmd)

