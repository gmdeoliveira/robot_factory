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
  gazebo_ros
  controller_manager
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