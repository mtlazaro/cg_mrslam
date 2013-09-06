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

#ifndef _CLOSURE_BUFFER_H_
#define _CLOSURE_BUFFER_H_

#include <list>

#include "g2o/core/optimizable_graph.h"


using namespace g2o;

struct VertexTime{
  int time;
  OptimizableGraph::Vertex* v;
};

struct VertexTimeComparator
{
  OptimizableGraph::Vertex* v;

VertexTimeComparator(OptimizableGraph::Vertex* v_)
: v(v_) {}
 
  bool operator()(const VertexTime& vt)
  {
    return vt.v->id() == v->id();
  }
};

struct ClosureBuffer {
  void addEdge(OptimizableGraph::Edge *e);
  void removeEdge(OptimizableGraph::Edge *e);

  void addEdgeSet(OptimizableGraph::EdgeSet& eset);
  void removeEdgeSet(OptimizableGraph::EdgeSet& eset);

  void addVertex(OptimizableGraph::Vertex *v);
  void removeVertex(OptimizableGraph::Vertex *v);

  OptimizableGraph::Vertex* findVertex(int idVertex);

  /* inline void setRobotId(int rid) { _robotId = rid;} */
  /* inline int robotId() const { return _robotId;} */

  inline OptimizableGraph::EdgeSet& edgeSet() {return _eset;}
  inline OptimizableGraph::VertexIDMap& vertices() {return _vmap;}

  inline std::list<VertexTime>& vertexList() {return _vertexList;}

  void updateList(int windowSize);
  bool checkList(int windowSize);

protected:
  std::list<VertexTime> _vertexList;
  OptimizableGraph::EdgeSet _eset;
  OptimizableGraph::VertexIDMap _vmap;
  //int _robotId;
};

#endif
