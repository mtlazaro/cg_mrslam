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

#include "vertices_finder.h"

VerticesFinder::VerticesFinder(OptimizableGraph *graph) {
  _graph = graph;
}

void VerticesFinder::findVerticesInDistance(OptimizableGraph::VertexSet& vset, OptimizableGraph::Vertex *currentVertex, double graphdist){
 
  HyperDijkstra hd(_graph);
  MyCostFunction mcf;
  hd.shortestPaths(currentVertex, &mcf, graphdist);
  
  vset=hd.visited();
}

void VerticesFinder::findVerticesLoopClosing(OptimizableGraph::VertexSet& vset, OptimizableGraph::Vertex *currentVertex, double graphdist){

  HyperDijkstra hd(_graph);
  MyCostFunction mcf;
  hd.shortestPaths(currentVertex, &mcf, graphdist);

  OptimizableGraph::VertexSet visited = hd.visited();

  for (OptimizableGraph::VertexIDMap::iterator it=_graph->vertices().begin(); it!=_graph->vertices().end(); ++it) {
    OptimizableGraph::Vertex* v= (OptimizableGraph::Vertex*)(it->second);

    OptimizableGraph::VertexSet::iterator itvs = visited.find(v);
    if (itvs == visited.end() && vertexDistance(currentVertex, v) <= MAX_EUC_DIST_LC )
       vset.insert(v);
  }
}

void VerticesFinder::findVerticesScanMatching(OptimizableGraph::Vertex *currentVertex, OptimizableGraph::VertexSet& vset){

  double distance = MAX_GRAPH_DIST_SM;
  
  findVerticesInDistance(vset, currentVertex, distance);

  OptimizableGraph::VertexSet vsetlc;
  distance = MIN_GRAPH_DIST_LC;
  findVerticesLoopClosing(vsetlc, currentVertex, distance);

  for (OptimizableGraph::VertexSet::iterator it = vsetlc.begin(); it != vsetlc.end(); it++){
    OptimizableGraph::Vertex* vertex = (OptimizableGraph::Vertex*) *it;
    vset.insert(vertex);
  }

  OptimizableGraph::VertexSet::iterator itv=vset.find(currentVertex);
  if (itv != vset.end()) //Remove currentVertex from the output vset
    vset.erase(itv);
}


void VerticesFinder::findSetsOfVertices(OptimizableGraph::VertexSet &mixedvset, std::set<OptimizableGraph::VertexSet> &setOfVSet){

  setOfVSet.clear();
  OptimizableGraph::VertexSet _mixedvset = mixedvset;
  while (!_mixedvset.empty()){
    HyperDijkstra hd(_graph);
    MyVSetCostFunction mcf(_mixedvset);
    OptimizableGraph::VertexSet::iterator itv = _mixedvset.begin();
    OptimizableGraph::Vertex* vertex = (OptimizableGraph::Vertex*) *itv;
    hd.shortestPaths(vertex, &mcf);
  
    OptimizableGraph::VertexSet vset = hd.visited();

    setOfVSet.insert(vset);
    for(OptimizableGraph::VertexSet::iterator it = vset.begin(); it!=vset.end(); it++)
      _mixedvset.erase(*it);
  }
}

OptimizableGraph::Vertex* VerticesFinder::findClosestVertex(OptimizableGraph::VertexSet &vset, OptimizableGraph::Vertex* currentVertex){
  double distance = std::numeric_limits<double>::max();

  OptimizableGraph::Vertex* closestVertex = 0; 
  for (OptimizableGraph::VertexSet::iterator it = vset.begin(); it != vset.end(); it++){
    OptimizableGraph::Vertex *v = (OptimizableGraph::Vertex*) *it;
    double d = vertexDistance(currentVertex, v);
    if (d < distance){
      distance = d;
      closestVertex = v;
    }
  }
  return closestVertex;
}
