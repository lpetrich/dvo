cmake_minimum_required(VERSION 2.8.3)
project(dvo_core)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)

message("cmake source directory: ${CMAKE_SOURCE_DIR}")

find_package(catkin REQUIRED COMPONENTS                                   
	ecl_linear_algebra
	ecl_time
	pcl_ros
	cmake_modules
	roscpp
)

find_package(sophus REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

set(SSE_FLAGS)
set(CMAKE_REQUIRED_FLAGS "-msse3") 
set(CMAKE_REQUIRED_FLAGS) 
set(SSE_FLAGS "-msse3 -mfpmath=sse") 
add_definitions(${SSE_FLAGS}) 

find_package(VTK REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(
		INCLUDE_DIRS include 
		LIBRARIES ${PROJECT_NAME}
		DEPENDS                
			sophus
		CATKIN_DEPENDS                
			ecl_linear_algebra
			ecl_time
		)

add_library(dvo_core
	src/core/interpolation.cpp
	src/core/intrinsic_matrix.cpp
	src/core/least_squares.cpp
	src/core/math_sse.cpp
	src/core/rgbd_image.cpp
	src/core/rgbd_image_sse.cpp
	src/core/point_selection.cpp
	src/core/surface_pyramid.cpp
	src/core/weight_calculation.cpp
	src/util/histogram.cpp
	src/visualization/camera_trajectory_visualizer.cpp
	src/dense_tracking.cpp
	src/dense_tracking_impl.cpp
	src/dense_tracking_config.cpp
)

target_link_libraries(dvo_core
	tbb
	boost_thread
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

add_library(dvo_visualization
	src/visualization/async_point_cloud_builder.cpp
	src/visualization/point_cloud_aggregator.cpp
	src/visualization/pcl_camera_trajetory_visualizer.cpp
)

target_link_libraries(dvo_visualization
	dvo_core
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

#add_executable(sse_test
#    src/sse_test.cpp
#    src/core/math_sse.cpp
#)

install(FILES 
	nodelet_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

message("Installing in ${CATKIN_PACKAGE_SHARE_DESTINATION}")