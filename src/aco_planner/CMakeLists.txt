cmake_minimum_required(VERSION 2.8.0)

project(aco_planner)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++11")   
endif(CMAKE_COMPILER_IS_GNUCXX)

set(catkin_deps
  roscpp
  std_msgs
  roslib
  )

find_package(
  catkin REQUIRED COMPONENTS
  ${catkin_deps}
  nav_msgs
  geometry_msgs
  message_generation
)
find_package(OpenCV 2.4 REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${catkin_deps}
  DEPENDS OpenCV
  nav_msgs
  geometry_msgs
)

add_executable(aco_planner_node
 src/aco_plan.cpp
 src/aco.cpp
 src/astar.cpp
 src/basicFuction.cpp)

target_link_libraries(aco_planner_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(aco_planner_talker
 src/aco_talker.cpp
 src/aco.cpp
 src/astar.cpp
 src/basicFuction.cpp
)

target_link_libraries(aco_planner_talker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS})

add_executable(aco_planner_listener
 src/aco_listener.cpp
)

target_link_libraries(aco_planner_listener
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS})



