#!/usr/bin/env bash
#
# Typical usage: ./join.bash dave_nvidia
#

IMG=$(basename $1)

# Get array of container id's based on ancestry
CONTAINERS=$(docker ps -aqf "ancestor=${IMG}")
#IFS=' '
#read -ra CONTAINER_IDS <<< "$CONTAINERS"
readarray -t CONTAINER_IDS <<<"$CONTAINERS"
N=${#CONTAINER_IDS[@]}

echo "$N Containers:"
for C in "${CONTAINER_IDS[@]}"
do
    echo "    <${C}>"
done

if [ $N -lt 1 ]
then
    echo "Cannot find any containers with the ancestry <${IMG}>"
    exit 1
elif [ $N -eq 1 ]
then
    CONTAINER_ID=$CONTAINER_IDS
    if [ ${#CONTAINER_ID} -lt 1 ]
    then
	echo "Cannot find any containers with the ancestry <${IMG}>"
	exit 1
    else
	echo "Found just one container <${CONTAINER_ID}>.  Joining..."
    fi
else
    CONTAINER_ID=${CONTAINER_IDS[0]}
    echo "Found $N containers with ancestor <$IMG>.  Joining the first one <$CONTAINER_ID}> ..."
fi


xhost +
docker exec --privileged -e DISPLAY=${DISPLAY} -e LINES=`tput lines` -it ${CONTAINER_ID} bash
xhost -
