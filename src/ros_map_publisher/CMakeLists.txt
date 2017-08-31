
add_library(cg_mrslam_ros_map_publisher_library
  frequency_map.cpp
  graph2occupancy.cpp
  grid_line_traversal.cpp
  occupancy_map_server.cpp
)

target_link_libraries(cg_mrslam_ros_map_publisher_library
  ${OpenCV_LIBS}    
)