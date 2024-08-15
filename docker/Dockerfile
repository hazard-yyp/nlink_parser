# Use the noetic-ros-base image as the base image
FROM ros:noetic-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone \
    && rm /etc/localtime \
    && ln -s /usr/share/zoneinfo/Asia/Singapore /etc/localtime \
    && apt-get update \
    && apt-get install -q -y --no-install-recommends tzdata

# Install basic apt tools and dependencies
RUN apt update && apt upgrade -y && apt install -y \
    vim git wget python3-rosdep build-essential cmake python3-catkin-tools python3-pip \  
    #apt-get install -y doxygen graphviz  && \
    && rm -rf /var/lib/apt/lists/*

# Setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic

# Source ROS setup scripts
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Create and build the catkin workspace
RUN pip install pyserial \
    && echo "Running catkin_make..." \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && mkdir -p /root/catkin_ws/src && cd /root/catkin_ws/src \
    && git clone --recursive https://github.com/hazard-yyp/nlink_parser.git \
    && cd /root/catkin_ws/src/nlink_parser/extern/serial \
    && make \
    && make install \
    && cd /root/catkin_ws \
    && catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set the working directory
WORKDIR /root/catkin_ws

CMD ["bash"]