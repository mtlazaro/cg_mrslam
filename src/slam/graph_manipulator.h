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

#ifndef _GRAPH_MANIPULATOR_H_
#define _GRAPH_MANIPULATOR_H_

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"

using namespace g2o;

struct GraphManipulator{
  GraphManipulator(SparseOptimizer* optimizer);
  void setVertices(OptimizableGraph::VertexSet& vertices); // vertices of interest to work with
  void setGauge(OptimizableGraph::VertexSet& gauge); // vertices to fix
  void setGauge(OptimizableGraph::Vertex* gauge);
  void setEdges(OptimizableGraph::EdgeSet& edges); //edges to include in the optimization

protected:
  void pushState(); // push vertices of the graph and save the state of each vertex (fixed/unfixed)
  void popState();  // pop vertices of the graph and restore the state of each vertex
  void fixGauge();
  void optimize();  // does the optimization of the graph
  OptimizableGraph::VertexSet _vertices;
  OptimizableGraph::VertexSet _gauge;
  OptimizableGraph::EdgeSet _edges;
  std::map<OptimizableGraph::Vertex*, bool> _fixes;
  bool _statePushed;
  SparseOptimizer* _optimizer;
};

/////////////////////////////////////////////////////
struct CovarianceEstimator: public GraphManipulator {
  CovarianceEstimator(SparseOptimizer* optimizer);
  void compute();
  MatrixXd getCovariance(OptimizableGraph::Vertex *v);

 protected:
  SparseBlockMatrix<MatrixXd> spinv;
};


#endif
