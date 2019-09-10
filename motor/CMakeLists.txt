cmake_minimum_required(VERSION 2.8.3)
project(motor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp dynamixel_sdk dynamixel_workbench_msgs dynamixel_workbench_toolbox message_filters pcl_ros laser_geometry sensor_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES automatico_mrs
  CATKIN_DEPENDS roscpp dynamixel_sdk dynamixel_workbench_msgs dynamixel_workbench_toolbox message_filters pcl_ros laser_geometry sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(mover_continuo src/mover_continuo.cpp)
target_link_libraries(mover_continuo ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(scanner src/scanner.cpp)
target_link_libraries(scanner ${catkin_LIBRARIES} ${PCL_LIBRARIES})


## Mark executables and/or libraries for installation
# install(TARGETS mover_continuo
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

