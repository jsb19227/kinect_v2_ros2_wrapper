## ROS2 Driver for Kinect V2 (Package and Read Me still work in progress)

### Requirements

ROS2 and libfreenect2

### Usage

Clone the repo into your ros2 workspace and then build

### Known issues

If CMake says it can't find libfreenect2, try running:

sudo ln -s $HOME/freenect2/lib/libfreenect2.so.0.2 /usr/lib/libfreenect2.so.0.2