FROM ros:foxy
COPY . /dev_ws/src/challenge_02
WORKDIR /dev_ws
RUN apt-get update
RUN apt-get install vim -y
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    echo "source install/setup.bash" >> /opt/ros/foxy/setup.bash && \
    colcon build
