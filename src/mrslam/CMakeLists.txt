add_subdirectory(condensed_graph)

add_library(cg_mrslam_library
  graph_comm.cpp
  msg_factory.cpp
  mr_graph_slam.cpp
  mr_closure_buffer.cpp
)
target_link_libraries(cg_mrslam_library
  cg_mrslam_condensed_graph_library
)
