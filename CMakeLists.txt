cmake_minimum_required(VERSION 2.8.3)
project(gazebo_closed_loop_hw_sim)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE RELEASE)

find_package(catkin REQUIRED COMPONENTS
  angles
  control_toolbox
  controller_manager
  gazebo_dev
  hardware_interface
  joint_limits_interface
  pluginlib
  roscpp
  std_msgs
  transmission_interface
  urdf
  gazebo_ros_control
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS
    angles
    control_toolbox
    controller_manager
    gazebo_dev
    hardware_interface
    joint_limits_interface
    pluginlib
    roscpp
    std_msgs
    transmission_interface
    urdf
    gazebo_ros_control
  INCLUDE_DIRS include
  LIBRARIES gazebo_closed_loop_hw_sim
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)

add_library(gazebo_closed_loop_hw_sim src/gazebo_closed_loop_hw_sim.cpp)
target_link_libraries(gazebo_closed_loop_hw_sim ${catkin_LIBRARIES})


#############
## Install ##
#############

INSTALL(TARGETS gazebo_closed_loop_hw_sim 
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

INSTALL(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

INSTALL(FILES gazebo_closed_loop_hw_sim_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)