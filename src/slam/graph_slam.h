// Copyright (c) 2013, Maria Teresa Lazaro Grañon
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this
//   list of conditions and the following disclaimer in the documentation and/or
//   other materials provided with the distribution.
//
//   Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef _GRAPH_SLAM_H_
#define _GRAPH_SLAM_H_

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "matcher/scan_matcher.h"
#include "vertices_finder.h"
#include "closure_checker.h"
#include "closure_buffer.h"

//#include "g2o/types/data/vertex_ellipse.h"

#include <boost/thread.hpp>

class GraphSLAM{


 public:
  GraphSLAM();

  void init(double resolution, double kernelRadius, int windowLoopClosure_, double maxScore, double inlierThreshold, int minInliers);

  void setIdRobot(int idRobot);
  inline int idRobot() const {return _idRobot;}
  void setBaseId(int baseId);
  inline int baseId() const {return _baseId;}
  bool isMyVertex(OptimizableGraph::Vertex *v);

  void setInitialData(SE2 initialTruePose, SE2 initialOdom, RobotLaser* laser); //For simulation
  void setInitialData(SE2 initialOdom, RobotLaser* laser);
  void addData(SE2 pose, RobotLaser* laser);
  void addDataSM(SE2 pose, RobotLaser* laser);

  inline SparseOptimizer *graph() {return _graph;}
  inline VertexSE2 *lastVertex() {return _lastVertex;}
  inline SE2 lastOdom() {return _lastOdom;}

  void findConstraints();

  void optimize(int nrunnings);
  bool saveGraph(const char *filename);
  bool loadGraph(const char *filename);

 protected:
 
  SparseOptimizer * _graph;
  int _idRobot;
  int _baseId;
  int _runningVertexId;
  int _runningEdgeId;

  VertexSE2 *_firstRobotPose;
  VertexSE2 *_lastVertex;
  SE2 _lastOdom;
 
  VerticesFinder _vf;
  ScanMatcher _closeMatcher;
  ScanMatcher _LCMatcher;

  ClosureBuffer _closures;

  int windowLoopClosure;
  double maxScore;
  double inlierThreshold;
  int minInliers;

  LoopClosureChecker lcc;

  Eigen::Matrix3d _odominf;
  Eigen::Matrix3d _SMinf;
  OptimizableGraph::EdgeSet _odomEdges;
  OptimizableGraph::EdgeSet _SMEdges;

  void addClosures(OptimizableGraph::EdgeSet loopClosingEdges);
  void checkClosures();
  void updateClosures();

  RobotLaser* findLaserData(OptimizableGraph::Vertex* v);
  //VertexEllipse* findEllipseData(OptimizableGraph::Vertex* v);

  void checkHaveLaser(OptimizableGraph::VertexSet& vset);
  void checkCovariance(OptimizableGraph::VertexSet& vset);
  void addNeighboringVertices(OptimizableGraph::VertexSet& vset, int gap);

  boost::mutex graphMutex;
};

#endif
