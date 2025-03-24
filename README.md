# cartesio_collision_avoidance

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](
https://opensource.org/licenses/BSD-3-Clause)
[![Ros Version](https://img.shields.io/badge/ROS1-Noetic-green)](
https://docs.ros.org/en/noetic/index.html)

| **Obstacle add to scene**                           |
|-------------------------------------------------------|
| <img src="https://github.com/hucebot/cartesio_collision_avoidance/blob/main/images/cartesian_obstacle.png" alt="Control Cartesio" width="240"> |

## Table of Contents
- [cartesio_collision_avoidance](#cartesio_collision_avoidance)
  - [Table of Contents](#table-of-contents)
  - [Get Started](#get-started)
    - [Installation](#installation)
      - [From Docker](#from-docker)
  - [Usage](#usage)
    - [ROS](#ros)
      - [ROS1](#ros1)

# Get Started

## Installation

### From Docker
The easiest way to get started is to use the provided Docker image. You can find the Dockerfile in the `docker` folder. To build the image, run the following command:

```bash
sh build.sh
```

To run the development container, use the following command:

```bash
sh run.sh
```

# Usage

## ROS

### ROS1

There are three launch file to run the obstacle avoidance node. The first one is `pointcloud_to_mesh_launcher.launch` which run to run the node to convert the pointcloud into a mesh. Then, the second launcher is `obstacle_launcher.launch` which run to run the mesh_viz, to visualize the obstacle in rviz. Finally, the third launcher is `tf_fix_launcher.launch` which run the transformation between the camera and the robot base frame.

```bash
roslaunch cartesio_collision_avoidance pointcloud_to_mesh_launcher.launch
```
```bash
roslaunch cartesio_collision_avoidance obstacle_launcher.launch
```
```bash
roslaunch cartesio_collision_avoidance tf_fix_launcher.launch
```