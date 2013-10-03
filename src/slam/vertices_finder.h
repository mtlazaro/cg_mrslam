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

#ifndef _VERTICES_FINDER_H_
#define _VERTICES_FINDER_H_

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/hyper_dijkstra.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

using namespace g2o;

inline double distanceSE2(SE2 r1, SE2 r2){
  Vector2d t = r1.translation()-r2.translation();
  return t.norm();
}

inline double vertexDistance(OptimizableGraph::Vertex* v1, OptimizableGraph::Vertex* v2){
  VertexSE2* v1se2 = dynamic_cast<VertexSE2*>(v1);
  VertexSE2* v2se2 = dynamic_cast<VertexSE2*>(v2);

  if (v1se2 && v2se2)
    return distanceSE2(v1se2->estimate(), v2se2->estimate());
  else
    return std::numeric_limits<double>::max();
}

struct MyCostFunction : public HyperDijkstra::CostFunction {
  virtual double operator() (HyperGraph::Edge* _e, HyperGraph::Vertex* _from, HyperGraph::Vertex* _to) {
    EdgeSE2* e=dynamic_cast<EdgeSE2*>(_e);
 
    if (!e)
      return std::numeric_limits<double>::max();
    
    VertexSE2* from=dynamic_cast<VertexSE2*>(_from);
    VertexSE2* to=dynamic_cast<VertexSE2*>(_to);
    /* std::cout << "From: " << from->id() << " = " << from->estimate().translation().x() << " " <<  from->estimate().translation().y() << " " << from->estimate().rotation().angle() << std::endl; */
    /* std::cout << "To: " << to->id() << " = " << to->estimate().translation().x() << " " <<  to->estimate().translation().y() << " " << to->estimate().rotation().angle() << std::endl; */

    Vector2d dt=from->estimate().translation()-to->estimate().translation();
   
    /* std::cout << "Distance: " << dt.norm() << std::endl; */
    return dt.norm();
  }
};

struct MyVSetCostFunction : public HyperDijkstra::CostFunction {

 MyVSetCostFunction(OptimizableGraph::VertexSet vset): _vset(vset){}
  virtual double operator() (HyperGraph::Edge* _e, HyperGraph::Vertex* _from, HyperGraph::Vertex* _to) {
    EdgeSE2* e=dynamic_cast<EdgeSE2*>(_e);
 
    if (!e)
      return std::numeric_limits<double>::max();
    
    OptimizableGraph::VertexSet::iterator it = _vset.find(_from);
    if(it == _vset.end())
       return std::numeric_limits<double>::max();

    it = _vset.find(_to);
    if(it == _vset.end())
       return std::numeric_limits<double>::max();

    return 1.0;
  }

  OptimizableGraph::VertexSet _vset;
};


#define MAX_GRAPH_DIST_SM 2.0 //Max distance in the graph to find neighbors for a vertex 
#define MIN_GRAPH_DIST_LC 5.0 //Min distance in the graph for a vertex to be considered for loop closing
#define MAX_EUC_DIST_LC 50.0 //Max euclidean distance for a vertex to be considered for loop closing 

class VerticesFinder{
 public:
  VerticesFinder(OptimizableGraph *graph);
 
  //Looks for vertices in the graph within a distance 'graphdist' from 'currentVertex'
  void findVerticesInDistance(OptimizableGraph::VertexSet& vset, OptimizableGraph::Vertex *currentVertex, double graphdist);
  //Looks for vertices in the graph further than a distance 'graphdist' from 'currentVertex'
  void findVerticesLoopClosing(OptimizableGraph::VertexSet& vset, OptimizableGraph::Vertex *currentVertex, double graphdist);
  //Looks for candidate vertices for scan matching
  void findVerticesScanMatching(OptimizableGraph::Vertex *currentVertex, OptimizableGraph::VertexSet& vset);
  //Groups the vertices in 'mixedvset' in sets of connected vertices
  void findSetsOfVertices(OptimizableGraph::VertexSet &mixedvset, std::set<OptimizableGraph::VertexSet> &setOfVSet);
  //Returns the closest vertex in 'vset' from 'currentVertex'
  OptimizableGraph::Vertex* findClosestVertex(OptimizableGraph::VertexSet &vset, OptimizableGraph::Vertex* currentVertex);

 protected:
  OptimizableGraph *_graph;
};

#endif
