#!/usr/bin/env bash

#
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

# Runs a docker container with the image created by build.bash
# Requires:
#   docker 19.03 or higher
#   nvidia-container-toolkit
#   an X server
# Recommended:
#   A joystick mounted to /dev/input/js0 or /dev/input/js1

GPUS="--gpus all"

POSITIONAL=()
while [[ $# -gt 0 ]]
do
key="$1"

case $key in
    -w|--without-nvidia)
    GPUS=""
    shift
    ;;
    *)    # unknown option
    POSITIONAL+=("$1")
    shift
    ;;
esac
done

set -- "${POSITIONAL[@]}"

if [ $# -lt 1 ]
then
    echo "Usage: $0 [-w --without-nvidia] <docker image> [<dir with workspace> ...]"
    exit 1
fi

IMG=$1

ARGS=("$@")
WORKSPACE=("${ARGS[@]:1}")

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -e $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Example: Bind mount a local repository on the host machine:
#DAVE_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../" >/dev/null 2>&1 && pwd )"
if [ -z $WORKSPACE ]; then
  WS_PATH=${HOME}/uuv_ws/src
else
  WS_PATH=$WORKSPACE
fi

DOCKER_OPTS="$GPUS --mount type=bind,source=${WS_PATH},target=/home/developer/uuv_ws/src"

# Share your vim settings.
# VIMRC=~/.vimrc
# if [ -f $VIMRC ]
# then
#   DOCKER_OPTS="$DOCKER_OPTS -v $VIMRC:/home/developer/.vimrc:ro"
# fi

USERID=$(id -u)
GROUPID=$(id -g)
docker run -it \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -v "$XAUTH:$XAUTH" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "/dev/input:/dev/input" \
  --privileged \
  --rm \
  --security-opt seccomp=unconfined \
  -u $USERID:$GROUPID \
  $DOCKER_OPTS \
  $IMG
