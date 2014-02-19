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

#include "graph_manipulator.h"

GraphManipulator::GraphManipulator(SparseOptimizer* optimizer) {
  _optimizer = optimizer;
  _statePushed = false;
  _vertices.clear();
  _gauge.clear();
  _edges.clear();
  _fixes.clear();
}

void GraphManipulator::setVertices(OptimizableGraph::VertexSet& vertices) {

  if (_statePushed)
    popState();

  _vertices = vertices;

}

void GraphManipulator::setGauge(OptimizableGraph::VertexSet& gauge) {
  _gauge = gauge;
}

void GraphManipulator::setGauge(OptimizableGraph::Vertex* gauge) {
  _gauge.clear();
  _gauge.insert(gauge);
}

void GraphManipulator::setEdges(OptimizableGraph::EdgeSet& edges) {
  _edges = edges;
}

void GraphManipulator::pushState() {

  assert(!_statePushed && "state already pushed");

  // save estimates and current state of the vertices
  for (OptimizableGraph::VertexIDMap::iterator it=_optimizer->vertices().begin(); it!=_optimizer->vertices().end(); ++it) {
    OptimizableGraph::Vertex* v= (OptimizableGraph::Vertex*)(it->second);
    v->push();
    _fixes.insert(std::make_pair(v, v->fixed()));
  }

  _statePushed = true;
}

void GraphManipulator::popState() {

  assert(_statePushed && "no vertices to be restored");

  // restore estimates and previous state of the vertices
  for (OptimizableGraph::VertexIDMap::iterator it=_optimizer->vertices().begin(); it!=_optimizer->vertices().end(); ++it) {
    OptimizableGraph::Vertex* v= (OptimizableGraph::Vertex*)(it->second);
    v->pop();
    v->setFixed(_fixes[v]);
  }

  _statePushed = false;
}

void GraphManipulator::fixGauge() {

  assert(_statePushed && "trying to fix non-saved vertices");
  /*
  //Fix vertices if in gauge, if not unfix
  for (OptimizableGraph::VertexSet::iterator it=_vertices.begin(); it!=_vertices.end(); ++it) {
    OptimizableGraph::Vertex* v= (OptimizableGraph::Vertex*)(*it);
    OptimizableGraph::VertexSet::iterator it = _gauge.find(v);
    if (it != _gauge.end())
      v->setFixed(true);
    else
      v->setFixed(false);
  }*/
  
  //Fix vertices if in gauge, if not unfix
  for (OptimizableGraph::VertexIDMap::iterator itv=_optimizer->vertices().begin(); itv!=_optimizer->vertices().end(); ++itv) {
    OptimizableGraph::Vertex* v= (OptimizableGraph::Vertex*)(itv->second);
    OptimizableGraph::VertexSet::iterator it = _gauge.find(v);
    if (it != _gauge.end())
      v->setFixed(true);
    else
      v->setFixed(false);
  }

}

void GraphManipulator::optimize() {
  if (_edges.size())
    _optimizer->initializeOptimization(_edges); //Optimize only this edges
  else
    _optimizer->initializeOptimization(); //Optimize all graph

  _optimizer->computeInitialGuess();
  _optimizer->optimize(1);
}
 
CovarianceEstimator::CovarianceEstimator(SparseOptimizer* optimizer) : GraphManipulator(optimizer){};

void CovarianceEstimator::compute() {
  pushState(); 
  fixGauge();
  optimize();
  
  //computeMarginals for the vertices
  std::vector<std::pair<int, int> > blockIndices;
  for (OptimizableGraph::VertexSet::iterator it =_vertices.begin(); it != _vertices.end(); it++){
    OptimizableGraph::Vertex* v = (OptimizableGraph::Vertex*)(*it);
    if (v->hessianIndex()>=0){
      blockIndices.push_back(std::make_pair(v->hessianIndex(), v->hessianIndex()));
    }
  }
  _optimizer->computeMarginals(spinv, blockIndices);
  
  popState();
}

MatrixXd CovarianceEstimator::getCovariance(OptimizableGraph::Vertex *v){
  OptimizableGraph::VertexSet::iterator it = _gauge.find(v);
  assert( (it == _gauge.end()) && "trying to get covariance from fixed vertex");
  it = _vertices.find(v);
  assert( (it != _vertices.end()) && "trying to get covariance from a vertex not contained in vertices");

  MatrixXd cov;
  if (v->hessianIndex()>=0)
    cov = *(spinv.block(v->hessianIndex(), v->hessianIndex()));
  return cov;
}

