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

#ifndef _LOOP_CLOSURE_CHECK_H_
#define _LOOP_CLOSURE_CHECK_H_

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

using namespace g2o;
using namespace std;

class LoopClosureChecker{
public:
  typedef std::map<OptimizableGraph::Edge*, double> EdgeDoubleMap;
  LoopClosureChecker();
  void init(OptimizableGraph::VertexIDMap& movableRegion, OptimizableGraph::EdgeSet& closingEdges, double inlierThreshold);
  inline int inliers() const {return _bestInliers;}
  inline EdgeDoubleMap& closures() {return _bestResult;}
  inline double chi2() const {return _bestChi2;}
  
  void check(const std::string& matchingType="2dPose");
protected:
  // this is gonna be unelegant
  void checkAndUpdateBest(OptimizableGraph::EdgeSet& eset, const std::string& matchingType="2dPose");

  void applyZeroErrorTransform(OptimizableGraph::EdgeSet& eset, 
			       const std::string& matchingType="2dPose");

  EdgeDoubleMap _bestResult;
  double        _bestChi2;
  int           _bestInliers;

  EdgeDoubleMap _tempResult;
  //EdgeDoubleMap _tempChi2;
  OptimizableGraph::VertexIDMap _localVertices;
  OptimizableGraph::EdgeSet   _closuresToCheck;
  double _inlierThreshold;
};

#endif
