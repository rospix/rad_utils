cmake_minimum_required(VERSION 2.8.3)
project(radiation_utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  rad_msgs
  mrs_lib
)

###########
## Eigen ##
###########
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen
  CATKIN_DEPENDS roscpp roslib std_msgs rad_msgs mrs_lib
  LIBRARIES physics marching_cubes
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(physics src/physics.cpp)
target_link_libraries(physics
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_library(marching_cubes src/marching_cubes.cpp)
target_link_libraries(marching_cubes
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_executable(visualization_test src/visualization_test.cpp)
target_link_libraries(visualization_test
  marching_cubes
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_executable(map_builder src/map_builder.cpp)
target_link_libraries(map_builder
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_library(gridmap src/gridmap.cpp)
target_link_libraries(gridmap
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_executable(gridmap_test src/gridmap_test.cpp)
target_link_libraries(gridmap_test
  gridmap
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
