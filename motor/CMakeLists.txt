cmake_minimum_required(VERSION 2.8.3)
project(motor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp dynamixel_sdk dynamixel_workbench_msgs dynamixel_workbench_toolbox
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES automatico_mrs
  CATKIN_DEPENDS roscpp dynamixel_sdk dynamixel_workbench_msgs dynamixel_workbench_toolbox
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
target_link_libraries(mover_continuo ${catkin_LIBRARIES})


## Mark executables and/or libraries for installation
# install(TARGETS mover_continuo
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

