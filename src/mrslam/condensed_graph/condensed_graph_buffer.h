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

#ifndef _CONDENSED_GRAPH_BUFFER_H_
#define _CONDENSED_GRAPH_BUFFER_H_

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/optimizable_graph.h"

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

#include "edge_labeler.h"
#include "condensed_graph_creator.h"

using namespace g2o;

struct CondensedGraphBuffer{
  CondensedGraphBuffer(SparseOptimizer* optimizer);
  
  void insertInClosure(int robot,  OptimizableGraph::VertexIDMap& vidmap);
  void insertOutClosure(int robot, OptimizableGraph::VertexIDMap& vidmap);
  
  OptimizableGraph::EdgeSet getMyEdges();
  OptimizableGraph::Vertex* selectOptimalGauge(OptimizableGraph::VertexIDMap* vertices);
  OptimizableGraph::Vertex* selectGauge(OptimizableGraph::VertexIDMap* vertices);
  OptimizableGraph::Vertex* selectGaugeCentroid(OptimizableGraph::VertexIDMap* vertices);

  void computeCondensedGraph(int robot, bool optimal = false);

  void insertEdgesFromRobot(int robot, OptimizableGraph::EdgeSet& eset);

  inline std::map<int, OptimizableGraph::VertexIDMap*>& inClosures(){return _inClosures;}
  inline std::map<int, OptimizableGraph::VertexIDMap*>& outClosures(){return _outClosures;}

  inline std::map<int, OptimizableGraph::EdgeSet>& outCondensedGraphs(){return _outCondensedGraphs;}

  inline std::map<int, OptimizableGraph::EdgeSet>& inCondensedGraphs(){return _inCondensedGraphs;}


  OptimizableGraph::VertexIDMap* inClosures(int robot);

  OptimizableGraph::VertexIDMap* outClosures(int robot);

  OptimizableGraph::EdgeSet outCondensedGraph(int robot);

  OptimizableGraph::EdgeSet inCondensedGraph(int robot);

protected:

  std::map<int, OptimizableGraph::VertexIDMap*> _inClosures;
  std::map<int, OptimizableGraph::VertexIDMap*> _outClosures;
  std::map<int, OptimizableGraph::EdgeSet> _outCondensedGraphs;
  std::map<int, OptimizableGraph::EdgeSet> _inCondensedGraphs;
  SparseOptimizer* _optimizer;
};

#endif
