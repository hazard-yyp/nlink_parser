#!/bin/zsh

# Set the container name
CONTAINER_NAME="uwb-ros"
NAMESPACE="robot_1"  # Define your desired namespace here

echo ">> Running container $CONTAINER_NAME with namespace $NAMESPACE..."
docker run -d --rm --name "$CONTAINER_NAME" --network=host --privileged \
    hazardyyp/nooploop_uwb_ros_driver:noetic \
    bash -c "source /root/catkin_ws/devel/setup.bash && \
            roslaunch nlink_parser multi_linktrack.launch namespace:=$NAMESPACE"
