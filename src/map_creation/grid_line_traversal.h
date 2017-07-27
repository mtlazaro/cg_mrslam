#pragma once

#include <vector>
#include <Eigen/Core>

#define GRIDTRAVERSAL_MAXPOINTS 65536

class  GridLineTraversalLine {
 public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  GridLineTraversalLine() {}
  virtual ~GridLineTraversalLine() {}
  
  int numPoints;
  Eigen::Vector2i points[GRIDTRAVERSAL_MAXPOINTS];
};

class GridLineTraversal {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  GridLineTraversal() {}
  virtual ~GridLineTraversal() {}

  static void gridLine(Eigen::Vector2i start, Eigen::Vector2i end, GridLineTraversalLine *line);
  static void gridLineCore(Eigen::Vector2i start, Eigen::Vector2i end, GridLineTraversalLine *line);
};
