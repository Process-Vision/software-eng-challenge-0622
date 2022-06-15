# Challenge 2

## Overview

In this challenge you are asked to create a ROS2 node running in a Docker container which will receive an input and calculate output values. The files that are provided contain build files for the ROS2 package (`CMakeLists.txt` and `package.xml`) and a C++ file (`src/node1.cpp`) that will serve as the basis for your code. Also included is a `Dockerfile` which lets you build a Docker image running Ubuntu 20.04 and ROS2 Foxy. Inside of here you will be able to compile and run the ROS2 node.

To complete this challenge, you will need to provide code that gives the correct output given any input. So, to simulate this, the task we look at is the projectile motion of a launched object. You will need to complete the trajectory of the object from somewhere mid-flight to the ground (z = 0). This mid-flight position and velocity will be given to the node from a ROS2 publisher and is already handled by the subscriber in the node. You should provide the logic that calculates points along the trajectory for each timestep and store the positions in the variables, `x`,`y` and `z`, which have been set up to publish to the ROS2 topic `/path`.

You can view the path that you are calculating using `ros2 topic echo /path` in your terminal, or if you are using an operating system which is supported you can view it using Rviz.

## Suggested Steps

- Build Docker image
- Create container using built image `docker build -t pv_ros .`
- Enter container `docker run -it --name challenge2 pv_ros` and modify `node1.cpp`
- Build the ROS2 node `colcon build`
- Run the ROS2 node `node1` using the following command `ros2 run challenge_02 node1`
- Create another session into the container [1]
- Start the 'ball rolling' by publishing the initial position and velocity parameters, to do so, run `ros2 topic pub /topic geometry_msgs/msg/PoseStamped "{pose:{position:{x: 0.0,y: 0.0, z: 1.0},orientation:{x: 3.3, y: 0.0,z: 5.0}}}" --once`. [2]

---

[1]: You may need to re-source the `setup.bash` by using the following `. install/setup.bash`
[2]: You may want to view the `path` topic, to do so, run `ros2 topic echo /path` in another session.
