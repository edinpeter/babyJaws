cmake_minimum_required(VERSION 2.8.3)
project(rov)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)

find_package(Boost REQUIRED COMPONENTS system)

add_executable(rov_node rov.cpp)

target_link_libraries(rov_node
  ${catkin_LIBRARIES}
  ${ROS_LIBRARIES}
)