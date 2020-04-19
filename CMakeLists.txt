cmake_minimum_required(VERSION 2.8.3)
project(radiation_utils)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dynamic_reconfigure
)

###########
## Eigen ##
###########
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})


#########################
## dynamic reconfigure ##
#########################
generate_dynamic_reconfigure_options(
  cfg/batch_visualizer.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen
  CATKIN_DEPENDS roscpp std_msgs
  LIBRARIES geometry batch_visualizer
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(geometry src/geometry.cpp)
target_link_libraries(geometry
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_library(batch_visualizer src/batch_visualizer.cpp)
target_link_libraries(batch_visualizer
  geometry
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

add_library(marching_cubes src/marching_cubes.cpp)
target_link_libraries(marching_cubes
  geometry
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
## Add configure headers for dynamic reconfigure
add_dependencies(batch_visualizer ${PROJECT_NAME}_gencfg)

add_executable(${PROJECT_NAME}_test src/test.cpp)
target_link_libraries(${PROJECT_NAME}_test  
  geometry
  batch_visualizer
  marching_cubes
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
