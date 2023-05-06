# PointCloud Filter (ROS2 package)
ROS2 package, build on humble distribution. This packages allows users to filter PointCloud data based on segmented image. Filtered classes can be set as ROS paramter as well as input clouds and segmented images (package accepts ROS PointCloud2, Image and CameraInfo topics). This package requires camera to be calibrated and have CameraInfo published in the same topic namespace as camera image (follow ROS2 camera naming conventions).

_Disclosure_: 
This package was created as a part of author's Bachelor's thesis.

## Dependecies

This package is intended to be used within the ROS2 enviroment, requires 
ROS2 package dependencies and works with ROS2 communication. Make sure you have ROS2 installed. You can choose any ROS2 distrubution (package was created on the "Humble" distro). To install ROS2 follow the proper documentation on this [link](https://docs.ros.org/en/humble/Installation.html "ROS2 install guide").


After installing ROS2 distrubution of your choice make sure all the required packages are installed.

### ROS2 dependencies:
You can install all required ROS2 dependecies by using `rosdep`:
```bash
# Run in the package base directory.
rosdep install -i --from-path src --rosdistro <ROS2 distribution> -y
```
### Non-ROS dependencies:
This package requires the following:
- [OpenCV](https://opencv.org/)
- [PCL 1.2+](https://pointclouds.org/)

## Building the package

After installing all the required dependencies you can build this package as any other ROS2 package by running `colcon`:
```bash
# Run in the package base directory.
colcon build --packages-select pointcloud_filter
```

## Launch

## Parameters explained
