# Turtlesim-Control-System
This is a ROS 2 project that implements:
- A user interface for controlling Turtlesim robots (turtle1 and turtle2) via velocity commands.
- A monitoring system that checks the distance between turtles and stops them if they are too close or near environment boundaries.

## Features
- Spawn and control multiple Turtlesim robots.
- Simple user interface for setting velocity commands.
- Automatic safety monitoring:
  - Stops turtles when they get too close to each other.
  - Stops turtles when they approach the boundaries of the environment.
## Setup and Usage

### Prerequisites
- ROS 2 (e.g., Humble, Galactic, or Foxy)
- `turtlesim` package installed.

### Building the Package
 Clone this repository into your ROS 2 workspace:
colcon build
source install/setup.bash


## Running the Nodes
ros2 run turtlesim turtlesim_node
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"

ros2 run assignment1_rt user_interface

ros2 run assignment1_rt distance_check

## Repository Organization in the Zip file
ros2-turtlesim-control/
├── assignment1_rt/
│   ├── __init__.py
│   ├── user_interface.py
│   ├── distance_check.py
├── setup.py
├── package.xml
├── README.md

