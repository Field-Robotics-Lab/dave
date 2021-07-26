#!/usr/bin/env bash
#
# Typical usage: ./join.bash <container_name> 
#

# Default container
CONTAINER_ID="dave_nvidia_runtime"
if [ "$#" -ne 1 ]; then
    echo "Joining default container <$CONTAINER_ID>"
else
    CONTAINER_ID=$1
fi

xhost +
docker exec --privileged -e DISPLAY=${DISPLAY} -e LINES=`tput lines` -it ${CONTAINER_ID} bash
xhost -
