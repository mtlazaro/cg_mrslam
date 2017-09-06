add_library(cg_mrslam_condensed_graph_library
  condensed_graph_buffer.cpp  
  condensed_graph_creator.cpp  
)

target_link_libraries(cg_mrslam_condensed_graph_library
  ${G2O_HIERARCHICAL_LIBRARY}
  )
