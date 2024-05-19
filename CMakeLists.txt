cmake_minimum_required(VERSION 3.5)
project(navyu_choreonoid)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(choreonoid REQUIRED)

set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(${CHOREONOID_INCLUDE_DIRS})
link_directories(${CHOREONOID_LIBRARY_DIRS})

choreonoid_add_simple_controller(DiffDriveController src/DiffDriveController.cpp)
target_link_libraries(DiffDriveController ${YAML_CPP_LIBRARIES})
ament_target_dependencies(DiffDriveController rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros tf2_eigen tf2_geometry_msgs)

ament_package()

install(DIRECTORY project launch config model rviz DESTINATION share/${PROJECT_NAME})
