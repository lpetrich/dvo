cmake_minimum_required(VERSION 2.8.3)
project(dvo_benchmark)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
  dvo_core
  dvo_ros
  roscpp
  pcl_ros
)

find_package(Eigen3 REQUIRED)

find_package(VTK REQUIRED)

catkin_package(
    INCLUDE_DIRS include 
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS dvo_core
)

include_directories(src include dvo_core dvo_ros ${catkin_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${VTK_INCLUDE_DIRS})

link_directories(${VTK_LIBRARY_DIRS})

#add_executable(benchmark
#     src/benchmark.cpp
#)

#add_executable(experiment
#     src/experiment.cpp
#)

#target_link_libraries(benchmark
#  dvo_core
#  dvo_ros
#  dvo_visualization
#  vtkCommon
#)

#target_link_libraries(experiment
#  dvo_core
#  dvo_visualization
#  vtkCommon
#)

# create output directory - just convenience for our users
file(MAKE_DIRECTORY "${PROJECT_SOURCE_DIR}/output")
file(MAKE_DIRECTORY "${PROJECT_SOURCE_DIR}/output/video")
