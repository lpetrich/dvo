cmake_minimum_required(VERSION 2.8.3)
project(dvo_tracking)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(message_generation)
find_package(catkin REQUIRED COMPONENTS 
	dynamic_reconfigure 
	dvo_core
	tf_conversions
	roscpp
	nodelet
	tf
	eigen_conversions
	pcl_ros
	sensor_msgs
	geometry_msgs
	image_transport
	message_filters
	cv_bridge
	visualization_msgs
	interactive_markers
	cmake_modules
)
set(INSTALL_DIR/usr/local)

generate_dynamic_reconfigure_options( 
	cfg/DvoTracking.cfg
)  

catkin_package(
		INCLUDE_DIRS include 
		LIBRARIES ${PROJECT_NAME}
		CATKIN_DEPENDS dynamic_reconfigure
)

include_directories(src include dvo_core ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} 
		src/camera_base.cpp
#		src/camera_dense_tracking.cpp
#		src/visualization/ros_camera_trajectory_visualizer.cpp
)
target_link_libraries(${PROJECT_NAME} 
		dvo_core
		dvo_visualization
		boost_thread
		${catkin_LIBRARIES}
		${Boost_LIBRARIES}
		${Eigen3_LIBRARIES}
)

#add_library(depth_tracker_nodelet
#		src/depth_tracker_nodelet.cpp
#)
#target_link_libraries(depth_tracker_nodelet
#	dvo_core
#	dvo_visualization
#		${PROJECT_NAME}
#		${catkin_LIBRARIES}
#)

#add_executable(depth_tracker
#		 src/depth_tracker_node.cpp
#)
#target_link_libraries(depth_tracker
#		dvo_core
#		dvo_visualization
#		${PROJECT_NAME}
#		${catkin_LIBRARIES}
#		${Boost_LIBRARIES}
#)

add_executable(depth_test
		 src/depth_test.cpp
)
target_link_libraries(depth_test
		dvo_core
		dvo_visualization
		${PROJECT_NAME}
		${catkin_LIBRARIES}
		${Boost_LIBRARIES}
)

add_executable(show_tracking
		 src/show_tracking.cpp
)
target_link_libraries(show_tracking
		${PROJECT_NAME}
		${catkin_LIBRARIES}
)

install(FILES 
	nodelet_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)