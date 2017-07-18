add_library(cg_mrslam_ros_utils_library
  ros_handler.cpp
  graph_ros_publisher.cpp)

target_link_libraries(cg_mrslam_ros_utils_library
 ${catkin_LIBRARIES}
)
add_dependencies(cg_mrslam_ros_utils_library cg_mrslam_generate_messages_cpp)
