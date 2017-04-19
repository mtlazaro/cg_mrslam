add_library(cg_mrslam_slam_library
  closure_buffer.cpp
  closure_checker.cpp
  graph_manipulator.cpp
  graph_slam.cpp
  vertices_finder.cpp
)

target_link_libraries(cg_mrslam_slam_library
  cg_mrslam_matcher_library
)