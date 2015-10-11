cmake_minimum_required(VERSION 2.8.3)
project(rov)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
roscpp
image_transport
cv_bridge
rosconsole
sensor_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} )

add_executable(rov_node src/rov.cpp)
add_executable(jaws_vision src/color_finder_average.cpp)
add_executable(lights_node src/robotLights.cpp)

target_link_libraries(lights_node
		${catkin_LIBRARIES}
		${ROS_LIBRARIES}
)

target_link_libraries(rov_node
  ${catkin_LIBRARIES}
  ${ROS_LIBRARIES}
)

target_link_libraries(jaws_vision 
	${catkin_LIBRARIES} 
	${OpenCV_LIBS} 
	${OpenCV_LIBRARIES} 
	${Eigen_LIBRARIES} 
	${QT_LIBRARIES} 
	${ROS_LIBRARIES} 
	${catkin_LIBRARIES}
)

