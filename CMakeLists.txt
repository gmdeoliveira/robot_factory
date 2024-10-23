cmake_minimum_required(VERSION 3.0.2)
project(robot_factory)

## Encontrar pacotes do ROS e Ignition
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  ros_ign_gazebo
  ros_ign_bridge
  gazebo_ros
  controller_manager
  ackermann_steering_controller
)

find_package(gazebo REQUIRED)

## Declarar o catkin_package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs gazebo_ros
)

## Incluir diretórios
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Instalar arquivos de launch
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)