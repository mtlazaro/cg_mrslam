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

#include "closure_buffer.h"

void ClosureBuffer::addEdge(OptimizableGraph::Edge *e){
    _eset.insert(e);
}

void ClosureBuffer::removeEdge(OptimizableGraph::Edge *e){
  OptimizableGraph::EdgeSet::iterator it = _eset.find(e);
  if (it != _eset.end())
    _eset.erase(it);
}

void ClosureBuffer::addEdgeSet(OptimizableGraph::EdgeSet& eset){
  for (OptimizableGraph::EdgeSet::iterator it=eset.begin(); it!=eset.end(); ++it){
    OptimizableGraph::Edge *e=(OptimizableGraph::Edge*)(*it);
    addEdge(e);
  }
}
  
void ClosureBuffer::removeEdgeSet(OptimizableGraph::EdgeSet& eset){
  for (OptimizableGraph::EdgeSet::iterator it=eset.begin(); it!=eset.end(); ++it){
    OptimizableGraph::Edge *e=(OptimizableGraph::Edge*)(*it);
    removeEdge(e);
  }
}

void ClosureBuffer::addVertex(OptimizableGraph::Vertex *v){
  _vmap.insert(std::make_pair(v->id(),v));
  VertexTime vt;
  vt.time = 0;
  vt.v = v;
  _vertexList.push_back(vt);
}

void ClosureBuffer::removeVertex(OptimizableGraph::Vertex *v){
  OptimizableGraph::VertexIDMap::iterator it=_vmap.find(v->id());
  if (it != _vmap.end()){
    _vmap.erase(it);
    
    OptimizableGraph::EdgeSet tmp = _eset;
    for (OptimizableGraph::EdgeSet::iterator it=tmp.begin(); it!=tmp.end(); ++it){
      OptimizableGraph::Edge *e=(OptimizableGraph::Edge*)(*it);
      
      for (size_t i=0; i<e->vertices().size(); i++){
	OptimizableGraph::Vertex* vedge=(OptimizableGraph::Vertex*)e->vertices()[i];
	if (vedge->id() == v->id())
	  removeEdge(e);
      }
    }
    _vertexList.remove_if(VertexTimeComparator(v));
  }
}

OptimizableGraph::Vertex* ClosureBuffer::findVertex(int idVertex){
  OptimizableGraph::VertexIDMap::iterator it=_vmap.find(idVertex);
  if (it == _vmap.end())
    return 0;

  OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(it->second);
  return v;
}


void ClosureBuffer::updateList(int windowSize){
  for(std::list<VertexTime>::iterator it = _vertexList.begin(); it!= _vertexList.end(); it++){
    (*it).time++;
  }
     
  std::list<VertexTime> tmp(_vertexList);
  for(std::list<VertexTime>::iterator it = tmp.begin(); it!= tmp.end(); it++){
    if ((*it).time >= windowSize) 
      removeVertex((*it).v);
  }
}

bool ClosureBuffer::checkList(int windowSize){
  for(std::list<VertexTime>::iterator it = _vertexList.begin(); it!= _vertexList.end(); it++){
    if ((*it).time == (windowSize-1))
      return true;
  }
  return false;  
}
