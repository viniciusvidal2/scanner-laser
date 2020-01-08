cmake_minimum_required(VERSION 2.8.3)
project(termica_reconstrucao)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(catkin REQUIRED cmake_modules COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
)

set(${PROJECT_NAME}_CATKIN_COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  message_filters
  dynamic_reconfigure
  stereo_msgs
  tf
  camera_calibration_parsers
  camera_info_manager
  image_geometry
)

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_COMPONENTS}
)

find_package(OpenCV)


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES termica_reconstrucao
  CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(reconstrucaoF ${PROJECT_SOURCE_DIR}/src/reconstrucaoF.cpp)
add_dependencies(reconstrucaoF ${catkin_EXPORTED_TARGETS})

target_link_libraries(reconstrucaoF
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Eigen_LIBRARIES}
)

## Vinicius
add_executable(projeta_termica src/projeta_termica.cpp)
add_dependencies(projeta_termica ${catkin_EXPORTED_TARGETS})
target_link_libraries (projeta_termica ${OpenCV_LIBS} ${Eigen_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(acumula_termica src/acumula_termica.cpp)
add_dependencies(acumula_termica ${catkin_EXPORTED_TARGETS})
target_link_libraries (acumula_termica ${OpenCV_LIBS} ${Eigen_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES} )


