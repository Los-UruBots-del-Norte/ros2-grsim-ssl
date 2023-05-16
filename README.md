# ROS2 GrSim SSL

This guide provides instructions on how to run grSim, messages and nodes to run a simulator for RoboCup Small Size League (SSL) robots.

## Prerequisites
- Ubuntu 22.04 or older
- C++ compiler (e.g., g++)
- CMake
- OpenGL
- Qt5
- Protobuf
- Git
- Colcon
- Pip
  
## Installation

1 - Create workspace
```
mkdir -p my_ros2_ws/src
colcon build
source install/setup.bash
```
2 - Clone this repository
 ```bash
git clone https://github.com/Los-UruBots-del-Norte/ros2-grsim-ssl.git my_ros2_ws/src
```
3 - Build grSim
```
cd grSim
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
make
```
4 - Install ssl-python-clients
```
cd my_ros2_ws/src/ssl-python-clients
pip install .
```
5 - Build Proto
```
cd my_ros2_ws/src/vision_comm/include/vision_comm/proto
./generate_proto.sh
``` 
6 - Build packages
```
colcon build --packages-select vision_comm grsim_ros_bridge grsim_ros_bridge_msgs krssg_ssl_msgs test_ssl
```

## Usage

Run the node launcher
```
source install/setup.bash
ros2 launch grsim_ros_bridge ssl.launch.py
```

## SSL example

Review the content of `test_ssl` package.


## Licence

Urubots







