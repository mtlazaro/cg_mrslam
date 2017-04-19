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

#include "condensed_graph_creator.h"

CondensedGraphCreator::CondensedGraphCreator(SparseOptimizer* optimizer) : GraphManipulator(optimizer){};

void CondensedGraphCreator::compute() {
  
  assert(_gauge.size() == 1 && "cannot create a condensed graph with multiple gauges");

  OptimizableGraph::VertexSet::iterator itg = _gauge.begin();
  OptimizableGraph::Vertex* gauge = (OptimizableGraph::Vertex*)(*itg); 

  OptimizableGraph::VertexSet::iterator itv = _vertices.find(gauge);
  assert( (itv != _vertices.end()) && "gauge must be contained in vertices");
  
  pushState();
  fixGauge();
  optimize();

  std::set<OptimizableGraph::Edge*> edgesToLabel;
  //Compute edges between the gauge and the rest of vertices
  for (OptimizableGraph::VertexSet::iterator it = _vertices.begin(); it != _vertices.end(); it++){
    OptimizableGraph::Vertex* v = (OptimizableGraph::Vertex*)(*it);
    if (v->id() != gauge->id()){
      EdgeSE2 *e = new EdgeSE2;
      e->vertices()[0] = gauge;
      e->vertices()[1] = v;

      _condensedGraph.insert(e);
      edgesToLabel.insert(e);
    }
  }

  //labelEdges
  EdgeLabeler labeler(_optimizer);
  labeler.labelEdges(edgesToLabel);

  popState();
}

