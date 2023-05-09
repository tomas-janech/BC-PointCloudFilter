cmake_minimum_required(VERSION 3.8)
project(projekcia-camera-matrix)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

add_library(projection PUBLIC src/Projector.cpp) 

add_executable(projekcia src/projekcia.cpp)

ament_target_dependencies(projection sensor_msgs image_geometry OpenCV pcl_ros pcl_conversions tf2_eigen tf2_ros tf2 rclcpp)
target_link_libraries(projekcia projection)
ament_target_dependencies(projekcia rclcpp sensor_msgs image_geometry cv_bridge OpenCV pcl_ros pcl_conversions)

INSTALL(TARGETS
  projekcia
  DESTINATION lib/${PROJECT_NAME})

ament_package()
