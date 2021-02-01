cmake_minimum_required(VERSION 3.1.2)
project(rad_utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  rad_msgs
  mrs_lib
)

find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  DEPENDS Eigen
  CATKIN_DEPENDS roscpp roslib std_msgs rad_msgs mrs_lib
  LIBRARIES physics marching_cubes RadUtilsGeometry
)

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

# add_library(elist_parser src/elist_parser.cpp)
# target_link_libraries(elist_parser
#   ${catkin_LIBRARIES}
#   ${Eigen_LIBRARIES}
# )

add_executable(visualization_test src/visualization_test.cpp)
target_link_libraries(visualization_test
  marching_cubes
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

# add_executable(elist_parser_test src/elist_parser_test.cpp)
# target_link_libraries(elist_parser_test
#   elist_parser
#   ${catkin_LIBRARIES}
#   ${Eigen_LIBRARIES}
# )

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

add_library(RadUtilsGeometry src/geometry.cpp)
target_link_libraries(RadUtilsGeometry
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
