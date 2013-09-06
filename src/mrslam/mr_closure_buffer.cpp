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
  
#include "mr_closure_buffer.h"

ClosureBuffer* MRClosureBuffer::findClosuresRobot(int robotId){
  std::map<int, ClosureBuffer*>::iterator it = mrClosures.find(robotId);
  if (it!=mrClosures.end()){
    ClosureBuffer* buffer=it->second;
    return buffer;
  }else 
    return 0;
}

void MRClosureBuffer::insert(ClosureBuffer& closures, int robotId){
  ClosureBuffer* previousBuffer = findClosuresRobot(robotId);
  if (previousBuffer){
    for (OptimizableGraph::VertexIDMap::iterator it =closures.vertices().begin(); it!=closures.vertices().end(); ++it){
      //For each vertex in closures
      OptimizableGraph::Vertex* vertexInClosures= (OptimizableGraph::Vertex*)(it->second);
      previousBuffer->addVertex(vertexInClosures);
    }

    for (OptimizableGraph::EdgeSet::iterator ite =closures.edgeSet().begin(); ite!=closures.edgeSet().end(); ++ite){
      //For each edge in closures
      previousBuffer->addEdge((OptimizableGraph::Edge*)*ite);
    }
  }else{
    mrClosures.insert(std::make_pair(robotId, new ClosureBuffer(closures)));
  }

}

void MRClosureBuffer::remove(ClosureBuffer& closures, int robotId){
  ClosureBuffer* previousBuffer = findClosuresRobot(robotId);
  if (previousBuffer){
    for (OptimizableGraph::VertexIDMap::iterator itv =closures.vertices().begin(); itv!=closures.vertices().end(); ++itv){
      //For each vertex in closures
      OptimizableGraph::Vertex* v= (OptimizableGraph::Vertex*)(itv->second);

      std::cout << "Removing vertex:" << v->id() << std::endl;
      previousBuffer->removeVertex(v);
    }

    for (OptimizableGraph::EdgeSet::iterator ite =closures.edgeSet().begin(); ite!=closures.edgeSet().end(); ++ite){
      //For each edge in closures
      
      //OptimizableGraph::Edge *e=(OptimizableGraph::Edge*)(*ite);
      //VertexSE2* vfrom=dynamic_cast<VertexSE2*>(e->vertices()[0]);
      //VertexSE2* vto=dynamic_cast<VertexSE2*>(e->vertices()[1]);
      //std::cout << "Removing Edge: From: " << vfrom->id() << " to: " << vto->id() << std::endl;

      previousBuffer->removeEdge((OptimizableGraph::Edge*)*ite);
    }

    if (previousBuffer->vertices().empty())
      mrClosures.erase(robotId);

  }
}

void MRClosureBuffer::update(int windowSize){

  std::map<int, ClosureBuffer*> tmp = mrClosures;

  for (std::map<int, ClosureBuffer*>::iterator it = tmp.begin(); it != tmp.end(); it++){
    ClosureBuffer* cb = it->second;

    cb->updateList(windowSize);   
    /*
    for (OptimizableGraph::VertexIDMap::iterator it = cb->vertices().begin(); it != cb->vertices().end(); it++){
      OptimizableGraph::Vertex* vertex= (OptimizableGraph::Vertex*)(it->second);
      std::cout << "Vertex in closures after update: " << vertex->id() << std::endl; 
    }
    for (OptimizableGraph::EdgeSet::iterator it = cb->edgeSet().begin(); it!=cb->edgeSet().end(); it++){
      OptimizableGraph::Edge* e=(OptimizableGraph::Edge*) (*it);
      OptimizableGraph::Vertex* vfrom=(OptimizableGraph::Vertex*)(e->vertices()[0]);
      OptimizableGraph::Vertex* vto=(OptimizableGraph::Vertex*)(e->vertices()[1]);
      std::cout << "Edge in closures after: From: " << vfrom->id() << " to: " << vto->id() << std::endl;
    }
    for(std::list<VertexTime>::iterator it = cb->vertexList().begin(); it!= cb->vertexList().end(); it++){
      VertexTime vt = *it;
      std::cout << "In list: Vertex: "  << vt.v->id() << " time: " << vt.time << std::endl;
    }
    */

    if (cb->vertices().empty())
      mrClosures.erase(it->first);

  }


}
