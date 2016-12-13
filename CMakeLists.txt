cmake_minimum_required(VERSION 2.8.3)
project(cg_mrslam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
   roscpp
   std_msgs
   message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Edge.msg
  Ping.msg
  RobotLaser.msg
  SLAM.msg
  VSE2.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cg_mrslam
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(cg_mrslam
#   src/${PROJECT_NAME}/cg_mrslam.cpp
# )

## Declare a cpp executable
# add_executable(cg_mrslam_node src/cg_mrslam_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(cg_mrslam_node cg_mrslam_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(cg_mrslam_node
#   ${catkin_LIBRARIES}
# )

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

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

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(srslam 
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
target_link_libraries(srslam 
  ${catkin_LIBRARIES}  
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
)
add_dependencies(srslam cg_mrslam_generate_messages_cpp)

add_executable(sim_mrslam
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
target_link_libraries(sim_mrslam
  ${catkin_LIBRARIES}  
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
)
add_dependencies(sim_mrslam cg_mrslam_generate_messages_cpp)

add_executable(real_mrslam
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
target_link_libraries(real_mrslam
  ${catkin_LIBRARIES}  
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
)
add_dependencies(real_mrslam cg_mrslam_generate_messages_cpp)

add_executable(bag_mrslam
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
target_link_libraries(bag_mrslam
  ${catkin_LIBRARIES}  
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
)
add_dependencies(bag_mrslam cg_mrslam_generate_messages_cpp)

add_executable(comm_publisher
  src/comm_publisher.cpp 
)
target_link_libraries(comm_publisher
  ${catkin_LIBRARIES}  
  ${G2O_STUFF_LIBRARY}
)
add_dependencies(comm_publisher cg_mrslam_generate_messages_cpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS cg_mrslam cg_mrslam_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cg_mrslam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
