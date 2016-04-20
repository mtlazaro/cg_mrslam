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

#include "condensed_graph_buffer.h"

SparseOptimizer* clone(SparseOptimizer* graph, int level){
  SparseOptimizer * subgraph = new SparseOptimizer;

  for (OptimizableGraph::EdgeSet::const_iterator it = graph->edges().begin(); it != graph->edges().end(); ++it) {
    EdgeSE2* e = dynamic_cast<EdgeSE2*>(*it);
    if (e->level() == level) {
      VertexSE2* vfrom = (VertexSE2*)(e->vertices()[0]);
      VertexSE2* v1 =  (VertexSE2*)subgraph->vertex(vfrom->id());
      if (!v1){
	v1 = new VertexSE2;
	v1->setId(vfrom->id());
	v1->setEstimate(vfrom->estimate());
	v1->setUserData(vfrom->userData());
	subgraph->addVertex(v1);
      }

      VertexSE2* vto = (VertexSE2*)(e->vertices()[1]);
      VertexSE2* v2 =  (VertexSE2*)subgraph->vertex(vto->id());
      if (!v2){
	v2 = new VertexSE2;
	v2->setId(vto->id());
	v2->setEstimate(vto->estimate());
	v2->setUserData(vto->userData());
	subgraph->addVertex(v2);
      }

      EdgeSE2* e2 = new EdgeSE2;      
      e2->vertices()[0] = v1;
      e2->vertices()[1] = v2;
      
      e2->setMeasurement(e->measurement());
      e2->setInformation(e->information());

      e2->setLevel(level);

      subgraph->addEdge(e2);
    }
  }
  
  return subgraph;
}

void removeSubgraph(SparseOptimizer* graph, int level){
  if (level > 0){
    OptimizableGraph::EdgeSet tmp(graph->edges());
    for (OptimizableGraph::EdgeSet::const_iterator it = tmp.begin(); it != tmp.end(); ++it) {
      OptimizableGraph::Edge* e = (OptimizableGraph::Edge*)(*it);
      if (e->level() == level)
	graph->removeEdge(e);
    }
  }
}
//////////////////////////////////////////////////////



CondensedGraphBuffer::CondensedGraphBuffer(SparseOptimizer* optimizer) {
  _optimizer = optimizer;
  _inClosures.clear();
  _outClosures.clear();
  _outCondensedGraphs.clear();
  _inCondensedGraphs.clear();
}

OptimizableGraph::VertexIDMap* CondensedGraphBuffer::inClosures(int robot){
  std::map<int, OptimizableGraph::VertexIDMap*>::iterator it = _inClosures.find(robot);
  if (it!=_inClosures.end()){
    OptimizableGraph::VertexIDMap* closures = it->second;
    return closures;
  }else
    return 0;
};

OptimizableGraph::VertexIDMap* CondensedGraphBuffer::outClosures(int robot){
  std::map<int, OptimizableGraph::VertexIDMap*>::iterator it = _outClosures.find(robot);
  if (it!=_outClosures.end()){
    OptimizableGraph::VertexIDMap* closures = it->second;
    return closures;
  }else
    return 0;
};

OptimizableGraph::EdgeSet CondensedGraphBuffer::outCondensedGraph(int robot){
  std::map<int, OptimizableGraph::EdgeSet>::iterator it = _outCondensedGraphs.find(robot);
  OptimizableGraph::EdgeSet cgraph;
  if (it!=_outCondensedGraphs.end())
    cgraph = it->second;
  return cgraph;
}

OptimizableGraph::EdgeSet CondensedGraphBuffer::inCondensedGraph(int robot){
  std::map<int, OptimizableGraph::EdgeSet>::iterator it = _inCondensedGraphs.find(robot);
  OptimizableGraph::EdgeSet eset;
  if (it!=_inCondensedGraphs.end())
    eset = it->second;
  return eset;
}



void CondensedGraphBuffer::insertInClosure(int robot, OptimizableGraph::VertexIDMap& vidmap){

  OptimizableGraph::VertexIDMap* invidmap = inClosures(robot);

  if (invidmap){
    for (OptimizableGraph::VertexIDMap::iterator itvmap = vidmap.begin(); itvmap != vidmap.end(); itvmap++){
      OptimizableGraph::VertexIDMap::iterator itv = invidmap->find(itvmap->first);
      
      if (itv == invidmap->end())
	invidmap->insert(*itvmap);
      /* else{
	VertexSE2* v = (VertexSE2*)(itvmap->second);
	VertexSE2* v2 = (VertexSE2*)(itv->second);
	
	v2->setEstimate(v->estimate());
	}*/
    }
  }else
    _inClosures.insert(std::make_pair(robot, new OptimizableGraph::VertexIDMap(vidmap)));
}

void CondensedGraphBuffer::insertOutClosure(int robot, OptimizableGraph::VertexIDMap& vidmap){

  OptimizableGraph::VertexIDMap* outvidmap = outClosures(robot);
  
  if (outvidmap){
    for (OptimizableGraph::VertexIDMap::iterator itvmap = vidmap.begin(); itvmap != vidmap.end(); itvmap++){
      OptimizableGraph::VertexIDMap::iterator itv = outvidmap->find(itvmap->first);
      if (itv == outvidmap->end())
	outvidmap->insert(*itvmap);
      /*else{
	VertexSE2* v = (VertexSE2*)(itvmap->second);
	VertexSE2* v2 = (VertexSE2*)(itv->second);
	  
	v2->setEstimate(v->estimate());
	}*/
    }
  }else
    _outClosures.insert(std::make_pair(robot,  new OptimizableGraph::VertexIDMap(vidmap)));
}

double computeOverallUncertainty(OptimizableGraph::EdgeSet& edges){

  double totalUncertainty = 0;
  for (OptimizableGraph::EdgeSet::iterator it = edges.begin(); it != edges.end(); it++){
    EdgeSE2* e = (EdgeSE2*)(*it);
    totalUncertainty += e->information().inverse().determinant();
  }
  return totalUncertainty;
}

/*
void saveFixedState(OptimizableGraph::VertexIDMap* vertices, std::map<int, bool>& state){
  state.clear();  
  for (OptimizableGraph::VertexIDMap::iterator it=vertices->begin(); it != vertices->end(); it++){
    OptimizableGraph::Vertex* v = (OptimizableGraph::Vertex*)(it->second);
    state.insert(std::make_pair(v->id(), v->fixed()));
  }
}

void restoreFixedState(OptimizableGraph::VertexIDMap* vertices, std::map<int, bool>& state){
  for (OptimizableGraph::VertexIDMap::iterator it=vertices->begin(); it != vertices->end(); it++){
    OptimizableGraph::Vertex* v = (OptimizableGraph::Vertex*)(it->second);
    v->setFixed(state[v->id()]);
  }
}*/

/*OptimizableGraph::Vertex* CondensedGraphBuffer::selectOptimalGauge(OptimizableGraph::VertexIDMap* vertices){

  EdgeLabeler labeler(_optimizer);

  std::map<int, bool> state;
  saveFixedState(vertices, state);

  OptimizableGraph::Vertex* bestGauge = 0;
  double bestUncertainty = std::numeric_limits<double>::max();
  OptimizableGraph::EdgeSet myOwnEdges = getMyEdges();  //I only consider my own information to build the condensed graph for the other robot
  for (OptimizableGraph::VertexIDMap::iterator it1=vertices->begin(); it1 != vertices->end(); it1++){
    OptimizableGraph::Vertex* candidateGauge = (OptimizableGraph::Vertex*)(it1->second);
    VertexSE2* vse2 = dynamic_cast<VertexSE2*>(candidateGauge);
    
    candidateGauge->setFixed(true);

    double totalUncertainty = 0;
    std::set<OptimizableGraph::Edge*> edgesToLabel;
    for (OptimizableGraph::VertexIDMap::iterator it2=vertices->begin(); it2 != vertices->end(); it2++){
      VertexSE2* v = (VertexSE2*)(it2->second);
      if (v->id() != vse2->id()){
	//Compute condensed graph, edges between Gauge and rest of vertices
	EdgeSE2 *e = new EdgeSE2;
	e->vertices()[0] = candidateGauge;
	e->vertices()[1] = v;

	v->setFixed(false);
    	edgesToLabel.insert(e);

      }
    }

    //initializeoptimization
    _optimizer->initializeOptimization(myOwnEdges);
    //optimize graph
    _optimizer->optimize(1);
    labeler.labelEdges(edgesToLabel);     //labelEdge

    totalUncertainty = computeOverallUncertainty(edgesToLabel);

    //std::cerr << "Vertex " << vse2->id() << " distance: " << totalUncertainty << std::endl;
   
    if (totalUncertainty < bestUncertainty){
      bestUncertainty = totalUncertainty;
      bestGauge = candidateGauge;
    }
  }

  restoreFixedState(vertices, state);
  std::cerr << "Best Gauge: " << bestGauge->id() << "Best uncertainty: " << bestUncertainty << std::endl;

  return bestGauge;
  }*/

OptimizableGraph::Vertex* CondensedGraphBuffer::selectOptimalGauge(OptimizableGraph::VertexIDMap* vertices){

  OptimizableGraph::Vertex* bestGauge = 0;
  double bestUncertainty = std::numeric_limits<double>::max();
  OptimizableGraph::EdgeSet myOwnEdges = getMyEdges();  //I only consider my own information to build the condensed graph for the other robot

  OptimizableGraph::VertexSet vset;
  for (OptimizableGraph::VertexIDMap::iterator it1=vertices->begin(); it1 != vertices->end(); it1++){
    OptimizableGraph::Vertex* v = (OptimizableGraph::Vertex*)(it1->second);
    vset.insert(v);    
  }

  for (OptimizableGraph::VertexIDMap::iterator it1=vertices->begin(); it1 != vertices->end(); it1++){
    OptimizableGraph::Vertex* candidateGauge = (OptimizableGraph::Vertex*)(it1->second);
   
    CondensedGraphCreator cgc = CondensedGraphCreator(_optimizer);
    cgc.setVertices(vset);
    cgc.setGauge(candidateGauge);
    cgc.setEdges(myOwnEdges);
    
    cgc.compute();
    OptimizableGraph::EdgeSet labeledEdges = cgc.getCondensedGraph();

    double totalUncertainty = computeOverallUncertainty(labeledEdges);

    //std::cerr << "Vertex " << candidateGauge->id() << " distance: " << totalUncertainty << std::endl;
   
    if (totalUncertainty < bestUncertainty){
      bestUncertainty = totalUncertainty;
      bestGauge = candidateGauge;
    }
  }

  std::cerr << "Best Gauge: " << bestGauge->id() << "Best uncertainty: " << bestUncertainty << std::endl;

  return bestGauge;
}

OptimizableGraph::Vertex* CondensedGraphBuffer::selectGauge(OptimizableGraph::VertexIDMap* vertices){
  
  OptimizableGraph::Vertex* bestGauge = 0;
  double bestDistance = std::numeric_limits<double>::max();
  for (OptimizableGraph::VertexIDMap::iterator it1=vertices->begin(); it1 != vertices->end(); it1++){
    OptimizableGraph::Vertex* candidateGauge = (OptimizableGraph::Vertex*)(it1->second);
    VertexSE2* vse2 = dynamic_cast<VertexSE2*>(candidateGauge);
    
    double currentDistance = 0;
    for (OptimizableGraph::VertexIDMap::iterator it2=vertices->begin(); it2 != vertices->end(); it2++){
      VertexSE2* v = (VertexSE2*)(it2->second);
      if (v->id() != vse2->id()){
	SE2 dt=vse2->estimate().inverse()*v->estimate();
	currentDistance += dt.translation().norm();
      }
    }
    //std::cerr << "Vertex " << vse2->id() << " distance: " << currentDistance << std::endl;

    if (currentDistance < bestDistance){
      bestDistance = currentDistance;
      bestGauge = candidateGauge;
    }
  }

  //std::cerr << "Best Gauge: " << bestGauge->id() << std::endl;
  return bestGauge;
}

OptimizableGraph::Vertex* CondensedGraphBuffer::selectGaugeCentroid(OptimizableGraph::VertexIDMap* vertices){
  
  OptimizableGraph::Vertex* bestGauge = 0;
  double bestDistance = std::numeric_limits<double>::max();
  Eigen::Vector2d sum(.0,.0);
  for (OptimizableGraph::VertexIDMap::iterator it=vertices->begin(); it != vertices->end(); it++){
       VertexSE2* v = (VertexSE2*)(it->second);
       sum += v->estimate().translation();
   }

  Eigen::Vector2d centroid(sum.x()/vertices->size(), sum.y()/vertices->size());

  for (OptimizableGraph::VertexIDMap::iterator it=vertices->begin(); it != vertices->end(); it++){
    OptimizableGraph::Vertex* candidateGauge = (OptimizableGraph::Vertex*)(it->second);
    VertexSE2* vse2 = dynamic_cast<VertexSE2*>(candidateGauge);
    
    Eigen::Vector2d vdist = vse2->estimate().translation() - centroid;
    double currentDistance = vdist.norm();

    if (currentDistance < bestDistance){
      bestDistance = currentDistance;
      bestGauge = candidateGauge;
    }
  }

  //std::cerr << "Best Gauge: " << bestGauge->id() << std::endl;
  return bestGauge;
}

OptimizableGraph::EdgeSet CondensedGraphBuffer::getMyEdges(){
  OptimizableGraph::EdgeSet myOwnEdges = _optimizer->edges();
  for (std::map<int, OptimizableGraph::EdgeSet>::iterator it = _inCondensedGraphs.begin(); it != _inCondensedGraphs.end(); it++){
    OptimizableGraph::EdgeSet edgesrobot = it->second;
    for (OptimizableGraph::EdgeSet::iterator ite = edgesrobot.begin(); ite != edgesrobot.end(); ite++){
      //Remove edge received from other robot
      OptimizableGraph::Edge* edge = (OptimizableGraph::Edge*) (*ite);
      myOwnEdges.erase(edge);
    }
  }
  //Also remove edges built for other robots 
  for (std::map<int, OptimizableGraph::EdgeSet>::iterator it = _outCondensedGraphs.begin(); it != _outCondensedGraphs.end(); it++){
    OptimizableGraph::EdgeSet edgesrobot = it->second;
    for (OptimizableGraph::EdgeSet::iterator ite =  edgesrobot.begin(); ite != edgesrobot.end(); ite++){
      OptimizableGraph::Edge* edge = (OptimizableGraph::Edge*) (*ite);
      myOwnEdges.erase(edge);
    }
  }
  return myOwnEdges;
}

/*void CondensedGraphBuffer::computeCondensedGraph(int robot, bool optimal){
  //See which vertices the other robot needs
  OptimizableGraph::VertexIDMap* outvidmap = outClosures(robot);
  if (outvidmap){
    OptimizableGraph::EdgeSet myOwnEdges = getMyEdges();  //I only consider my own information to build the condensed graph for the other robot 
    int level = robot+1;
    //Remove if we already have a subgraph of this level
    removeSubgraph(_optimizer, level);

    //Save state
    std::map<int, bool> state;
    saveFixedState(outvidmap, state);

    //Select one gauge among them and fix it
    OptimizableGraph::Vertex* gauge = 0;
    if (optimal)
      gauge = selectOptimalGauge(outvidmap);
    else
    gauge = selectGauge(outvidmap);

    // OptimizableGraph::VertexIDMap::iterator it1=outvidmap->begin();
    // OptimizableGraph::Vertex* gauge = (OptimizableGraph::Vertex*)(it1->second);


    //bool wasFixed = gauge->fixed(); //save if the vertex was already fixed
    gauge->setFixed(true);
      
    OptimizableGraph::EdgeSet edges;
    std::set<OptimizableGraph::Edge*> edgesToLabel;
    //Compute edges between the gauge and the rest of vertices
    for (OptimizableGraph::VertexIDMap::iterator it = outvidmap->begin(); it != outvidmap->end(); it++){
      VertexSE2* v = (VertexSE2*)(it->second);
      if (v->id() != gauge->id()){
	EdgeSE2 *e = new EdgeSE2;
	e->vertices()[0] = gauge;
	e->vertices()[1] = v;
	v->setFixed(false);
	e->setLevel(level);

	//Add them to my graph with a level higher than 0 (for example, the id of the other robot)
	_optimizer->addEdge(e);
	edgesToLabel.insert(e);
	edges.insert(e);
      }
    }	

    //initializeoptimization
    _optimizer->initializeOptimization(myOwnEdges);
    //optimize graph
    _optimizer->optimize(1);
    //labelEdges
    EdgeLabeler labeler(_optimizer);
    labeler.labelEdges(edgesToLabel);
	
    //Restore previous status of the vertex
    restoreFixedState(outvidmap, state);
    //gauge->setFixed(wasFixed);
    //std::cerr << "Gauge: " << gauge->id() << std::endl;
    //std::cerr << "Total Uncertainty: " << computeOverallUncertainty(edgesToLabel) << std::endl;
    std::map<int, OptimizableGraph::EdgeSet>::iterator itg = _outCondensedGraphs.find(robot);
    if (itg!= _outCondensedGraphs.end())
      _outCondensedGraphs.erase(itg);
      
    _outCondensedGraphs.insert(std::make_pair(robot, edges));
      
  }//else we cannot build the subgraph if we don't know which vertices the other robot needs

  }*/

void CondensedGraphBuffer::computeCondensedGraph(int robot, bool optimal){
  //See which vertices the other robot needs
  OptimizableGraph::VertexIDMap* outvidmap = outClosures(robot);
  if (outvidmap){
    OptimizableGraph::EdgeSet myOwnEdges = getMyEdges();  //I only consider my own information to build the condensed graph for the other robot 
    int level = robot+1;
    //Remove if we already have a subgraph of this level
    removeSubgraph(_optimizer, level);

    CondensedGraphCreator gc = CondensedGraphCreator(_optimizer);
    OptimizableGraph::VertexSet subGraph;
    for (OptimizableGraph::VertexIDMap::iterator it = outvidmap->begin(); it != outvidmap->end(); it++){
      OptimizableGraph::Vertex* v = (OptimizableGraph::Vertex*)(it->second);
      subGraph.insert(v);
    }

    gc.setVertices(subGraph);
    
    //Select one gauge among them and fix it
    OptimizableGraph::Vertex* gauge = 0;
    if (optimal)
      gauge = selectOptimalGauge(outvidmap);
    else
      gauge = selectGaugeCentroid(outvidmap);
    
    gc.setGauge(gauge);
    gc.setEdges(myOwnEdges);

    gc.compute();
    
    OptimizableGraph::EdgeSet labeledEdges = gc.getCondensedGraph();
    //Add computed edges to the graph with a different level depending on the robot
    for (OptimizableGraph::EdgeSet::iterator it = labeledEdges.begin(); it != labeledEdges.end(); it++) {
      OptimizableGraph::Edge* edge = (OptimizableGraph::Edge*) (*it);
      edge->setLevel(level);
      _optimizer->addEdge(edge);
    }

    //std::cerr << "Gauge: " << gauge->id() << std::endl;
    //std::cerr << "Total Uncertainty: " << computeOverallUncertainty(labeledEdges) << std::endl;
    std::map<int, OptimizableGraph::EdgeSet>::iterator itg = _outCondensedGraphs.find(robot);
    if (itg!= _outCondensedGraphs.end())
      _outCondensedGraphs.erase(itg);
      
    _outCondensedGraphs.insert(std::make_pair(robot, labeledEdges));
      
  }//else we cannot build the subgraph if we don't know which vertices the other robot needs

}

void CondensedGraphBuffer::insertEdgesFromRobot(int robot, OptimizableGraph::EdgeSet& eset){
  std::map<int, OptimizableGraph::EdgeSet>::iterator itr = _inCondensedGraphs.find(robot);
 
  if (itr != _inCondensedGraphs.end()){
    //Replace old edges from robot by the new ones received
    OptimizableGraph::EdgeSet oldEdges = itr->second;
    //Remove old edges from graph
    for(OptimizableGraph::EdgeSet::iterator ite = oldEdges.begin(); ite != oldEdges.end(); ite++){
      OptimizableGraph::Edge* e=(OptimizableGraph::Edge*)(*ite);
      _optimizer->removeEdge(e);
    }
    _inCondensedGraphs.erase(itr);
  } 

  _inCondensedGraphs.insert(std::make_pair(robot, eset));
    
  
  //Add new edges to graph
  for(OptimizableGraph::EdgeSet::iterator ite = eset.begin(); ite != eset.end(); ite++){
    OptimizableGraph::Edge* e=(OptimizableGraph::Edge*)(*ite);
    _optimizer->addEdge(e);
  }
  
}

