cmake_minimum_required(VERSION 2.8.3)
project(ltu_actor_graph)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(example src/example.cpp)
add_executable(twist src/twist.cpp)
target_link_libraries(twist ${catkin_LIBRARIES})
add_executable(std_msgs src/std_msgs.cpp)
target_link_libraries(std_msgs ${catkin_LIBRARIES})
