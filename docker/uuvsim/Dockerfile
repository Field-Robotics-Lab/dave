# Ubuntu 18.04 with nvidia-docker2 beta opengl support.
ARG BASEIMG=ubuntu:bionic
FROM $BASEIMG

# Set ROS distribution
ARG DIST=melodic

# Set Gazebo verison
ARG GAZ=gazebo9

# Tools useful during development.
RUN apt update \
 && apt install -y \
        build-essential \
        cppcheck \
        curl \
        cmake \
        lsb-release \
        gdb \
        git \
        mercurial \
        python3-dbg \
        python3-pip \
        python3-venv \
        ruby \
        software-properties-common \
        sudo \
        vim \
        wget \
        libeigen3-dev \
        pkg-config \
        protobuf-compiler \
 && apt clean

RUN export DEBIAN_FRONTEND=noninteractive \
 && apt update \
 && apt install -y \
    tzdata \
 && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt clean

# Add ROS melodic and Gazebo 9 to apt
RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
 && /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
 && apt update \
 && apt clean

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    python-catkin-tools

# install ROS and Gazebo dependencies
RUN apt install -y \
    qtbase5-dev \
    libgles2-mesa-dev \
    ${GAZ} \
    lib${GAZ}-dev \
    ros-${DIST}-ros-base \
    ros-${DIST}-xacro \
    ros-${DIST}-gazebo-ros \
    ros-${DIST}-gazebo-plugins \
    ros-${DIST}-hector-gazebo-plugins \
    ros-${DIST}-joy \
    ros-${DIST}-joy-teleop \
    ros-${DIST}-key-teleop \
    ros-${DIST}-robot-localization \
    ros-${DIST}-robot-state-publisher \
    ros-${DIST}-rviz \
    ros-${DIST}-teleop-tools \
    ros-${DIST}-teleop-twist-keyboard \
    ros-${DIST}-velodyne-simulator \
    ros-${DIST}-rqt \
    ros-${DIST}-rqt-common-plugins \
		ros-${DIST}-effort-controllers \
		ros-${DIST}-control-toolbox \
 		ros-${DIST}-controller-manager \
		ros-${DIST}-ros-control \
		python-scipy \
		ros-${DIST}-joint-state-publisher \
		ros-${DIST}-urdfdom-py \
		python-lxml \
		ros-${DIST}-ros-controllers \
		ros-${DIST}-kdl-parser-py \
		ros-${DIST}-gazebo-ros-control \
		ros-${DIST}-gazebo-ros-pkgs

RUN rosdep init \
    && rosdep update

# Set USER and GROUP
ARG USER=developer
ARG GROUP=developer

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`.

RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

RUN addgroup --gid 1000 $USER && \
    adduser --uid 1000 --ingroup $USER --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER

RUN adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# Commands below run as the developer user.
USER $USER:$GROUP

# When running a container start in the developer's home folder.
WORKDIR /home/$USER

# Create workspace
RUN mkdir -p uuv_ws/src/uuv_simulator

# Copy the VRX repository from the local file system
# We can't use the USER:GROUP variables until Docker adds support to --chown
#COPY --chown=developer:developer . uuv_ws/src/uuv_simulator

# Compile the VRX project.
#RUN /bin/bash -c ". /opt/ros/${DIST}/setup.bash && cd uuv_ws && catkin_make"

# Source all the needed environment files.
RUN /bin/sh -c 'echo ". /opt/ros/${DIST}/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc'
# && /bin/sh -c 'echo ". ~/uuv_ws/devel/setup.sh" >> ~/.bashrc'

ENTRYPOINT ["fixuid"]

CMD ["/bin/bash"]
# Customize your image here.


