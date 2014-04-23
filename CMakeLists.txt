cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

FIND_PACKAGE(OpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_PWN_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

FIND_PACKAGE(SuiteSparse REQUIRED)
INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

# Find g2o
FIND_PACKAGE(G2O REQUIRED)
IF(G2O_FOUND)
  SET(${PROJECT_NAME}_G2O_INCLUDE ${G2O_INCLUDE_DIR} CACHE PATH "Directory of G2O include")
  SET(${PROJECT_NAME}_G2O_LIBRARY_PATH ${G2O_LIBRARY_PATH} CACHE PATH "Directory of G2O libraries")
ELSE(G2O_FOUND)
  SET(${PROJECT_NAME}_G2O_INCLUDE "" CACHE PATH "Directory of G2O include")
ENDIF(G2O_FOUND)

include_directories(${G2O_INCLUDE_DIR})

include_directories(src)

rosbuild_add_executable(sim_mrslam
  src/sim_mrslam.cpp 
  src/ros_handler.cpp
  src/graph_ros_publisher.cpp
  src/matcher/scan_matcher.cpp
  src/matcher/chargrid.cpp
  src/slam/closure_checker.cpp
  src/slam/closure_buffer.cpp
  src/slam/vertices_finder.cpp
  src/slam/graph_manipulator.cpp
  src/slam/graph_slam.cpp
  src/mrslam/mr_graph_slam.cpp
  src/mrslam/msg_factory.cpp
  src/mrslam/mr_closure_buffer.cpp
  src/mrslam/graph_comm.cpp
  src/mrslam/condensedGraphs/condensed_graph_buffer.cpp
  src/mrslam/condensedGraphs/edge_labeler.cpp
  src/mrslam/condensedGraphs/condensed_graph_creator.cpp
  )
target_link_libraries(
  sim_mrslam
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
  )

rosbuild_add_executable(real_mrslam 
  src/real_mrslam.cpp 
  src/ros_handler.cpp
  src/graph_ros_publisher.cpp
  src/matcher/scan_matcher.cpp
  src/matcher/chargrid.cpp
  src/slam/closure_checker.cpp
  src/slam/closure_buffer.cpp
  src/slam/vertices_finder.cpp
  src/slam/graph_manipulator.cpp
  src/slam/graph_slam.cpp
  src/mrslam/mr_graph_slam.cpp
  src/mrslam/msg_factory.cpp
  src/mrslam/mr_closure_buffer.cpp
  src/mrslam/graph_comm.cpp
  src/mrslam/condensedGraphs/condensed_graph_buffer.cpp
  src/mrslam/condensedGraphs/edge_labeler.cpp
  src/mrslam/condensedGraphs/condensed_graph_creator.cpp
  )
target_link_libraries(
  real_mrslam
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
  )

rosbuild_add_executable(bag_mrslam
  src/bag_mrslam.cpp 
  src/ros_handler.cpp
  src/graph_ros_publisher.cpp
  src/matcher/scan_matcher.cpp
  src/matcher/chargrid.cpp
  src/slam/closure_checker.cpp
  src/slam/closure_buffer.cpp
  src/slam/vertices_finder.cpp
  src/slam/graph_manipulator.cpp
  src/slam/graph_slam.cpp
  src/mrslam/mr_graph_slam.cpp
  src/mrslam/msg_factory.cpp
  src/mrslam/mr_closure_buffer.cpp
  src/mrslam/graph_comm.cpp
  src/mrslam/condensedGraphs/condensed_graph_buffer.cpp
  src/mrslam/condensedGraphs/edge_labeler.cpp
  src/mrslam/condensedGraphs/condensed_graph_creator.cpp
  )
target_link_libraries(
  bag_mrslam
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
  )

rosbuild_add_executable(srslam 
  src/srslam.cpp 
  src/ros_handler.cpp
  src/graph_ros_publisher.cpp
  src/matcher/scan_matcher.cpp
  src/matcher/chargrid.cpp
  src/slam/closure_checker.cpp
  src/slam/closure_buffer.cpp
  src/slam/vertices_finder.cpp
  src/slam/graph_manipulator.cpp
  src/slam/graph_slam.cpp
  src/mrslam/msg_factory.cpp
  )
target_link_libraries(
  srslam
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
  )