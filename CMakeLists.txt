cmake_minimum_required(VERSION 2.8.3)
project(scene_recognition)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
add_definitions(-std=c++11 -g -O3 -ftree-vectorize -march=native)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_filters
  pcl_conversions
)

find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
catkin_package()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES scene_recognition
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(

  ${catkin_INCLUDE_DIRS}
)
include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)




add_executable(scenerecognition src/scenerecognition.cpp )
target_link_libraries(scenerecognition ${PCL_LIBRARIES} ${catkin_LIBRARIES})
