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

#include "mr_graph_slam.h"


MRGraphSLAM::MRGraphSLAM() :condensedGraphs(GraphSLAM::graph()), 
			    maxScoreMR(GraphSLAM::maxScore), 
			    minInliersMR(GraphSLAM::minInliers), 
			    windowMRLoopClosure(GraphSLAM::windowLoopClosure){

  factory = new MessageFactory();

  factory->registerMessageType<ComboMessage>();
  factory->registerMessageType<CondensedGraphMessage>();
  factory->registerMessageType<GraphMessage>();
  factory->registerMessageType<EdgeArrayMessage>();
  factory->registerMessageType<VertexArrayMessage>();
  factory->registerMessageType<RobotLaserMessage>();
			    }

void MRGraphSLAM::setIdRobot(int idRobot){
  GraphSLAM::setIdRobot(idRobot);
}

void MRGraphSLAM::setInterRobotClosureParams(double maxScoreMR_, int minInliersMR_, int windowMRLoopClosure_){
  maxScoreMR = maxScoreMR_;
  minInliersMR = minInliersMR_;
  windowMRLoopClosure = windowMRLoopClosure_;

}

void MRGraphSLAM::checkInterRobotClosures(){
  
  if (interRobotClosures.size()){
    cout << endl << "Inter Robot Closure Checking." << endl;

    for (std::map<int, ClosureBuffer*>::iterator it = interRobotClosures.mrClosures.begin(); it != interRobotClosures.mrClosures.end(); it++){
      int robotId = it->first;
      ClosureBuffer* cb = it->second;
      std::cout << "Closures robot " << robotId << std::endl;
      if (cb->checkList(windowMRLoopClosure)){
	lcc.init(cb->vertices(), cb->edgeSet(), inlierThreshold);
	lcc.check();

	cout << "Best Chi2 = " << lcc.chi2() << endl;
	cout << "Inliers = " << lcc.inliers() << endl;

	if (lcc.inliers() >= minInliersMR){
	  OptimizableGraph::VertexIDMap inClosures;
	  LoopClosureChecker::EdgeDoubleMap results = lcc.closures();
	  cout << "Results:" << endl;
	  for (LoopClosureChecker::EdgeDoubleMap::iterator it= results.begin(); it!= results.end(); it++){
	    EdgeSE2* e = (EdgeSE2*) (it->first);
	    VertexSE2* vto=dynamic_cast<VertexSE2*>(e->vertices()[1]);
	    cout << "Edge from: " << e->vertices()[0]->id() << " to: " << vto->id() << ". Chi2 = " << it->second <<  endl;

	    if (it->second < inlierThreshold){
	      cout << "Is an inlier. Adding to Graph" << endl;
	   
	      e->setId(++_runningEdgeId + _baseId); 
	      _graph->addEdge(e);
	      OptimizableGraph::Vertex* inserted = _graph->vertex(vto->id());
	      if (!inserted)
		_graph->addVertex(vto);
	      else{
		//I have this vertex in the graph... maybe I don't have the laser added
		RobotLaser* laserv = findLaserData(inserted);
		if (!laserv){
		  RobotLaser* laservto = findLaserData(vto);		    
		  inserted->setUserData(laservto);
		}
	      }
	      inClosures.insert(std::make_pair(vto->id(), vto));
	    }
	  }
	  cout << endl;
	  if (inClosures.size())
	    condensedGraphs.insertInClosure(robotId, inClosures);
	}
      }
      
    }
  }
}

void MRGraphSLAM::updateInterRobotClosures(){
  interRobotClosures.update(windowMRLoopClosure);
}

void MRGraphSLAM::addInterRobotData(ComboMessage* cmsg, OptimizableGraph::Vertex* refVertex){

  OptimizableGraph::VertexSet vset;
  for (size_t i = 0; i< cmsg->vertexVector.size(); i++){
    int vID = cmsg->vertexVector[i].id;
    SE2 vest(cmsg->vertexVector[i].estimate[0], cmsg->vertexVector[i].estimate[1], cmsg->vertexVector[i].estimate[2]);

    //See if the vertex is already in the graph
    VertexSE2 *v1 =  dynamic_cast<VertexSE2 *>(graph()->vertex(vID));    
    if (v1)
      continue;
     
    //See if the vertex is in the list of loop closures.
    ClosureBuffer* cb = interRobotClosures.findClosuresRobot(cmsg->robotId());
    if (cb){
      OptimizableGraph::Vertex* v2 = cb->findVertex(vID);
      if (v2){
	//Update estimate
	VertexSE2 *vse2 =  dynamic_cast<VertexSE2 *>(v2);
	vse2->setEstimate(vest);
	vset.insert(vse2);
	continue;
      }
    }

    //See if the vertex is in the list of non matched vertices.
    ClosureBuffer* cv = interRobotVertices.findClosuresRobot(cmsg->robotId());
    if (cv){
      OptimizableGraph::Vertex* v2 = cv->findVertex(vID);
      if (v2){
	//Update estimate
	VertexSE2 *vse2 =  dynamic_cast<VertexSE2 *>(v2);
	vse2->setEstimate(vest);
	vset.insert(vse2);
	continue;
      }
    }

    if (vID == cmsg->nodeId){
      //New vertex received
      VertexSE2* v = new VertexSE2;
      v->setId(cmsg->nodeId); //new node received with laser info
      //  LaserParameters lparams(0, cmsg->readings.size(), cmsg->minangle,   cmsg->angleincrement,  cmsg->maxrange, cmsg->accuracy, 0);
      LaserParameters lparams(0, cmsg->readings.size(), cmsg->minangle,   cmsg->angleincrement, 8.0, cmsg->accuracy, 0);
      RobotLaser* robotlaser = new RobotLaser;
      robotlaser->setLaserParams(lparams);
      robotlaser->setRanges(cmsg->readings);
      v->setUserData(robotlaser);
      v->setEstimate(vest);
      robotlaser->setOdomPose(vest);
      vset.insert(v);
    }
  }

  OptimizableGraph::VertexSet referenceVset;
  VertexSE2 *referenceVertex = (VertexSE2*) refVertex;
  referenceVset.insert(referenceVertex);
  int gap = 10;
  for (int j = 1; j <= gap; j++){
    VertexSE2 *vj =  dynamic_cast<VertexSE2 *>(graph()->vertex(referenceVertex->id()-j));
    if (vj)
      referenceVset.insert(vj);
    else
      break;
  }
  for (int j = 1; j <= gap; j++){
    VertexSE2 *vj =  dynamic_cast<VertexSE2 *>(graph()->vertex(referenceVertex->id()+j));
    if (vj)
      referenceVset.insert(vj);
    else
      break;
  }

  cout << "Vertices for global matching" << endl;
  for (OptimizableGraph::VertexSet::iterator it = referenceVset.begin(); it!= referenceVset.end(); it++){
    cout << (*it)->id() << " ";
  }
  cout << endl;
  for (OptimizableGraph::VertexSet::iterator it = vset.begin(); it!= vset.end(); it++){
    cout << (*it)->id() << " ";
  }
  cout << endl;

  
  if (vset.size()){
    VertexSE2 *v = new VertexSE2;
    for (OptimizableGraph::VertexSet::iterator itv = vset.begin(); itv != vset.end(); itv++){
      VertexSE2 *vertex = (VertexSE2*) *itv;
      if (vertex->id() == cmsg->nodeId){
	v = vertex;
	break;
      }
    }

    SE2 transf;
    bool shouldIAdd = _LCMatcher.globalMatching(referenceVset, referenceVertex, vset, v, &transf, maxScoreMR);
    
    if (shouldIAdd){
      cerr << "Found inter robot match" << endl;
      double score;
      bool robotDetected = _LCMatcher.verifyMatching(referenceVset, referenceVertex, vset, v, transf, &score); 
      if (robotDetected){
	//cerr << "Adding edge from " << referenceVertex->id() << " to " << v->id() 
	//     << " Estimate: "  << transf.translation().x() << " " << transf.translation().y() << " " << transf.rotation().angle() << endl;

	EdgeSE2 *ne = new EdgeSE2;
	ne->vertices()[0] = referenceVertex;
	ne->vertices()[1] = v;
      
	ne->setMeasurement(transf);
      
	Eigen::Matrix3d inf = 100 * Eigen::Matrix3d::Identity();
	inf(2,2) = 1000;
	ne->setInformation(inf);
      
	ClosureBuffer closure;
	closure.addVertex(v);
	closure.addEdge(ne);

	interRobotClosures.insert(closure, cmsg->robotId());
      }else{cerr << "Not Robot Detected in Range" << endl;}
 
    }  else {
      ClosureBuffer c;
      c.addVertex(v);

      interRobotVertices.insert(c, cmsg->robotId());    
    }
  }

  //findInterRobotConstraints();
}

void MRGraphSLAM::findInterRobotConstraints(){
  boost::mutex::scoped_lock lockg(graphMutex);

  OptimizableGraph::VertexSet referenceVset;
  VertexSE2 *referenceVertex = (VertexSE2*) lastVertex();
  referenceVset.insert(referenceVertex);
  
  int gap = 20;
  for (int j = 1; j <= gap; j++){
    VertexSE2 *vj =  dynamic_cast<VertexSE2 *>(graph()->vertex(referenceVertex->id()-j));
    if (vj)
      referenceVset.insert(vj);
    else
      break;
  }

  MRClosureBuffer tmp = interRobotVertices;
  //Try to match the non-matched vertices from all robots
  for (std::map<int, ClosureBuffer*>::iterator it = tmp.mrClosures.begin(); it != tmp.mrClosures.end(); it++){
    int robotId = it->first;
    ClosureBuffer *c = it->second;
    cerr << "Trying to match vertices from robot: " << robotId << endl;
    OptimizableGraph::VertexIDMap vertices = c->vertices();
    cerr << "Vertices to match: "; 
    for (OptimizableGraph::VertexIDMap::iterator itv = vertices.begin(); itv != vertices.end(); itv++){
      VertexSE2* vertex= (VertexSE2*)(itv->second);
      cerr << vertex->id() << " " ; 
    }
    cerr << std::endl;
    for (OptimizableGraph::VertexIDMap::iterator it2 = vertices.begin(); it2 != vertices.end(); it2++){
      VertexSE2* v = (VertexSE2*) (it2->second);

      SE2 transf;
      bool shouldIAdd = _LCMatcher.globalMatching(referenceVset, referenceVertex, v, &transf, maxScoreMR);
      if (shouldIAdd){
	cerr << "Found match with vertex " << v->id() << endl;
	double score;
	OptimizableGraph::VertexSet vset;
	vset.insert(v);
	bool robotDetected = _LCMatcher.verifyMatching(referenceVset, referenceVertex, vset, v, transf, &score); 
	if (robotDetected){
	  //If matched, remove from interRobotVertices, add to interRobotClosures
	  EdgeSE2 *ne = new EdgeSE2;
	  ne->vertices()[0] = referenceVertex;
	  ne->vertices()[1] = v;
      
	  ne->setMeasurement(transf);

	  Eigen::Matrix3d inf = 100 * Eigen::Matrix3d::Identity();
	  inf(2,2) = 1000;
	  ne->setInformation(inf);

	  ClosureBuffer closure;
	  closure.addVertex(v);
	  closure.addEdge(ne);

	  interRobotClosures.insert(closure, robotId);
	  interRobotVertices.remove(closure, robotId);
	}else {cerr << "Not Robot Detected in Range" << endl;}
      }
    }
    
  }

  //Update list of closures(window)
  checkInterRobotClosures();
  updateInterRobotClosures();
  interRobotVertices.update(windowMRLoopClosure);
}

void MRGraphSLAM::addInterRobotData(CondensedGraphMessage* gmsg){
  ClosuresMessage* cmsg = dynamic_cast<ClosuresMessage*>(gmsg);
  //cerr << "Adding inter Robot Data CondensedGraphMessage" << endl;
  if (cmsg){
    OptimizableGraph::VertexIDMap closures;
    //cerr << "Robot " << gmsg->robotId() << " asked for node: " << endl;
    for (size_t i=0; i<cmsg->closures.size(); i++){
      int nodeId = cmsg->closures[i];
      //cerr << "Node: " << nodeId << endl;
      OptimizableGraph::Vertex* v=graph()->vertex(nodeId);
      if (v)
	closures.insert(std::make_pair(nodeId,v));
    }

    if (closures.size()){
      condensedGraphs.insertOutClosure(gmsg->robotId(), closures);
      condensedGraphs.computeCondensedGraph(gmsg->robotId());
    }

  }

  EdgeArrayMessage* emsg = dynamic_cast<EdgeArrayMessage*>(gmsg);
  if (emsg){
    OptimizableGraph::EdgeSet edges;
    
    for (size_t i=0; i<emsg->edgeVector.size(); i++){
      int idFrom = emsg->edgeVector[i].idfrom; 
      int idTo = emsg->edgeVector[i].idto;

      OptimizableGraph::Vertex* vfrom=graph()->vertex(idFrom);
      OptimizableGraph::Vertex* vto=graph()->vertex(idTo);

      if (vfrom && vto){
	EdgeSE2 *e = new EdgeSE2;
	e->vertices()[0] = vfrom;
	e->vertices()[1] = vto;

	SE2 est(emsg->edgeVector[i].estimate[0], 
		emsg->edgeVector[i].estimate[1],
		emsg->edgeVector[i].estimate[2]);

	//cerr << "Adding subgraph edge from " << idFrom << " to " << idTo << " Estimate: " << emsg->edgeVector[i].estimate[0] << " " << emsg->edgeVector[i].estimate[1] << " " << emsg->edgeVector[i].estimate[2] << endl;

	Eigen::Matrix3d inf;
	inf(0,0) = emsg->edgeVector[i].information[0];
	inf(0,1) = emsg->edgeVector[i].information[1];
	inf(0,2) = emsg->edgeVector[i].information[2];
	inf(1,1) = emsg->edgeVector[i].information[3];
	inf(1,2) = emsg->edgeVector[i].information[4];
	inf(2,2) = emsg->edgeVector[i].information[5];
	inf(1,0) = inf(0,1);
	inf(2,0) = inf(0,2);
	inf(2,1) = inf(1,2);

	e->setMeasurement(est);
	e->setInformation(inf);
	
	edges.insert(e);
      }
    }
    
    if (edges.size())
      condensedGraphs.insertEdgesFromRobot(gmsg->robotId(), edges);
  }
}

void MRGraphSLAM::addInterRobotData(GraphMessage* gmsg){
  ClosuresMessage* cmsg = dynamic_cast<ClosuresMessage*>(gmsg);
  //cerr << "Adding inter Robot Data GraphMessage" << endl;
  if (cmsg){
    OptimizableGraph::VertexIDMap closures;
    for (size_t i=0; i<cmsg->closures.size(); i++){
      int nodeId = cmsg->closures[i];
      OptimizableGraph::Vertex* v=graph()->vertex(nodeId);
      if (v)
	closures.insert(std::make_pair(nodeId,v));
    }

    if (closures.size()){
      condensedGraphs.insertOutClosure(gmsg->robotId(), closures);
      condensedGraphs.computeCondensedGraph(gmsg->robotId());
    }

  }

  VertexArrayMessage* vmsg = dynamic_cast<VertexArrayMessage*>(gmsg);
  if (vmsg){
    cerr << "Adding vertices from robot: "; 
    for (size_t i=0; i<vmsg->vertexVector.size(); i++){
      VertexSE2* v=(VertexSE2*)(graph()->vertex(vmsg->vertexVector[i].id));
      SE2 est(vmsg->vertexVector[i].estimate[0],
	      vmsg->vertexVector[i].estimate[1],
	      vmsg->vertexVector[i].estimate[2]);
      if (v){
	if (!isMyVertex(v))
	  //Update estimate
	  v->setEstimate(est);
      }else{
	v = new VertexSE2;
	v->setId(vmsg->vertexVector[i].id);
	v->setEstimate(est);
	cerr << v->id() << " " ;
	graph()->addVertex(v);
      }
      
    }
    cerr << endl;
  }
    

  EdgeArrayMessage* emsg = dynamic_cast<EdgeArrayMessage*>(gmsg);
  if (emsg){
    OptimizableGraph::EdgeSet edges;
    
    for (size_t i=0; i<emsg->edgeVector.size(); i++){
      int idFrom = emsg->edgeVector[i].idfrom; 
      int idTo = emsg->edgeVector[i].idto;

      OptimizableGraph::Vertex* vfrom=graph()->vertex(idFrom);
      OptimizableGraph::Vertex* vto=graph()->vertex(idTo);

      if (vfrom && vto){
	EdgeSE2 *e = new EdgeSE2;
	e->vertices()[0] = vfrom;
	e->vertices()[1] = vto;

	SE2 est(emsg->edgeVector[i].estimate[0], 
		emsg->edgeVector[i].estimate[1],
		emsg->edgeVector[i].estimate[2]);

	Eigen::Matrix3d inf;
	inf(0,0) = emsg->edgeVector[i].information[0];
	inf(0,1) = emsg->edgeVector[i].information[1];
	inf(0,2) = emsg->edgeVector[i].information[2];
	inf(1,1) = emsg->edgeVector[i].information[3];
	inf(1,2) = emsg->edgeVector[i].information[4];
	inf(2,2) = emsg->edgeVector[i].information[5];
	inf(1,0) = inf(0,1);
	inf(2,0) = inf(0,2);
	inf(2,1) = inf(1,2);

	e->setMeasurement(est);
	e->setInformation(inf);
	
	edges.insert(e);
      }
    }
    
    if (edges.size())
      condensedGraphs.insertEdgesFromRobot(gmsg->robotId(), edges);
  }

}

void MRGraphSLAM::addInterRobotData(StampedRobotMessage vmsg){
  boost::mutex::scoped_lock lockg(graphMutex);
  std::cerr << "Adding inter robot data " << std::endl;
  ComboMessage* cmsg = dynamic_cast<ComboMessage*>(vmsg.msg);
  if (cmsg)
    addInterRobotData(cmsg, vmsg.refVertex);
  else{
    CondensedGraphMessage* cgmsg = dynamic_cast<CondensedGraphMessage*>(vmsg.msg);
    if (cgmsg)
      addInterRobotData(cgmsg);
    else{
      GraphMessage* gmsg = dynamic_cast<GraphMessage*>(vmsg.msg);
     if (gmsg)
       addInterRobotData(gmsg);
    }
  }
}

VertexArrayMessage* MRGraphSLAM::constructVertexArrayMessage(OptimizableGraph::VertexIDMap& vertices){
  RobotMessage* msg = factory->constructMessage(1);
  VertexArrayMessage* vmsg = dynamic_cast<VertexArrayMessage*>(msg);
  vmsg->setRobotId(idRobot());

  if (vertices.size()){
    vmsg->vertexVector.resize(vertices.size());
    int i = 0;
    for (OptimizableGraph::VertexIDMap::const_iterator it = vertices.begin(); it != vertices.end(); ++it) {
      VertexSE2* v= dynamic_cast<VertexSE2*>(it->second);

      vmsg->vertexVector[i].id = v->id();
      vmsg->vertexVector[i].estimate[0] = v->estimate().translation().x();
      vmsg->vertexVector[i].estimate[1] = v->estimate().translation().y();
      vmsg->vertexVector[i].estimate[2] = v->estimate().rotation().angle();

      i++;
    }

    return vmsg;
  }
  return 0;
}

EdgeArrayMessage* MRGraphSLAM::constructEdgeArrayMessage(OptimizableGraph::EdgeSet& edges){
  RobotMessage* msg = factory->constructMessage(5);
  EdgeArrayMessage* emsg = dynamic_cast<EdgeArrayMessage*>(msg);
  emsg->setRobotId(idRobot());

  if (edges.size()){
    emsg->edgeVector.resize(edges.size());
    int i = 0;
    for (OptimizableGraph::EdgeSet::const_iterator it = edges.begin(); it != edges.end(); ++it) {
      EdgeSE2* e = dynamic_cast<EdgeSE2*>(*it);
      //cerr << "Sending Edge from: " << e->vertices()[0]->id() << " to: " << e->vertices()[1]->id() << endl;
      
      VertexSE2* vfrom =  (VertexSE2*)(e->vertices()[0]);
      VertexSE2* vto =  (VertexSE2*)(e->vertices()[1]);

      emsg->edgeVector[i].idfrom = vfrom->id();
      emsg->edgeVector[i].idto = vto->id();
      emsg->edgeVector[i].estimate[0] = e->measurement().translation().x();
      emsg->edgeVector[i].estimate[1] = e->measurement().translation().y();
      emsg->edgeVector[i].estimate[2] = e->measurement().rotation().angle();

      Eigen::Matrix3d inf = e->information();
      emsg->edgeVector[i].information[0] = inf(0,0);
      emsg->edgeVector[i].information[1] = inf(0,1);
      emsg->edgeVector[i].information[2] = inf(0,2);
      emsg->edgeVector[i].information[3] = inf(1,1);
      emsg->edgeVector[i].information[4] = inf(1,2);      
      emsg->edgeVector[i].information[5] = inf(2,2);

      i++;
    }

    return emsg;
  }
  return 0;
}

ComboMessage* MRGraphSLAM::constructComboMessage(){
  boost::mutex::scoped_lock lockg(graphMutex);

  RobotMessage* msg = factory->constructMessage(4);
  ComboMessage* cmsg = dynamic_cast<ComboMessage*>(msg);
  cmsg->setRobotId(idRobot());

  //Fill in vertexArray info
  int nVertices = 5;
  OptimizableGraph::VertexIDMap vertices;
  vertices.insert(make_pair(lastVertex()->id(), lastVertex()));

  int i = 1;
  while (i <nVertices){
    int vID = lastVertex()->id()-i;
    VertexSE2 *v =  dynamic_cast<VertexSE2 *>(graph()->vertex(vID));
    if (v){
      vertices.insert(make_pair(v->id(), v));
    }else
      break;
    i++;
  }

  VertexArrayMessage* vmsg = dynamic_cast<VertexArrayMessage*>(cmsg);
  VertexArrayMessage* cvmsg = constructVertexArrayMessage(vertices);

  if (cvmsg)
    *vmsg = *cvmsg;
  
  //Fill in laser info
  RobotLaser* robotLaser = findLaserData(lastVertex());
  if (robotLaser){
    cmsg->readings = robotLaser->ranges();
    cmsg->nodeId = lastVertex()->id();
    cmsg->minangle = robotLaser->laserParams().firstBeamAngle;
    cmsg->angleincrement = robotLaser->laserParams().angularStep;
    cmsg->maxrange = robotLaser->laserParams().maxRange;
    cmsg->accuracy = robotLaser->laserParams().accuracy;
  }
  
  return cmsg;
}

CondensedGraphMessage* MRGraphSLAM::constructCondensedGraphMessage(int idRobotTo){
  boost::mutex::scoped_lock lockg(graphMutex);

  RobotMessage* msg = factory->constructMessage(7);
  CondensedGraphMessage* gmsg = dynamic_cast<CondensedGraphMessage*>(msg);
  gmsg->setRobotId(idRobot());

  OptimizableGraph::VertexIDMap* inClosuresRobot = condensedGraphs.inClosures(idRobotTo);
  if (inClosuresRobot){
    gmsg->closures.resize(inClosuresRobot->size());
    int i = 0;
    for (OptimizableGraph::VertexIDMap::iterator it = inClosuresRobot->begin(); it != inClosuresRobot->end(); it++){
      int vId = it->first;
      //cerr << "Asking for node " << vId << endl;
      gmsg->closures[i] = vId;
      i++;
    }
  }

  EdgeArrayMessage* emsg = dynamic_cast<EdgeArrayMessage*>(gmsg);

  OptimizableGraph::EdgeSet subgraphRobot = condensedGraphs.outCondensedGraph(idRobotTo);
  EdgeArrayMessage* cemsg = constructEdgeArrayMessage(subgraphRobot);

  if (cemsg)
    *emsg = *cemsg;
 
  /*
    OptimizableGraph::EdgeSet* subgraphRobot = condensedGraphs.outCondensedGraph(idRobotTo);
    if (subgraphRobot){
    gmsg->edgeVector.resize(subgraphRobot->size());
    int i = 0;
    for (OptimizableGraph::EdgeSet::const_iterator it = subgraphRobot->begin(); it != subgraphRobot->end(); ++it) {
    EdgeSE2* e = dynamic_cast<EdgeSE2*>(*it);
    //cerr << "Sending Edge from: " << e->vertices()[0]->id() << " to: " << e->vertices()[1]->id() << endl;
      
    VertexSE2* vfrom =  (VertexSE2*)(e->vertices()[0]);
    VertexSE2* vto =  (VertexSE2*)(e->vertices()[1]);

    gmsg->edgeVector[i].idfrom = vfrom->id();
    gmsg->edgeVector[i].idto = vto->id();
    gmsg->edgeVector[i].estimate[0] = e->measurement().translation().x();
    gmsg->edgeVector[i].estimate[1] = e->measurement().translation().y();
    gmsg->edgeVector[i].estimate[2] = e->measurement().rotation().angle();

    Matrix3d inf = e->information();
    gmsg->edgeVector[i].information[0] = inf(0,0);
    gmsg->edgeVector[i].information[1] = inf(0,1);
    gmsg->edgeVector[i].information[2] = inf(0,2);
    gmsg->edgeVector[i].information[3] = inf(1,1);
    gmsg->edgeVector[i].information[4] = inf(1,2);      
    gmsg->edgeVector[i].information[5] = inf(2,2);

    i++;
    }
    }*/
  
  //if (inClosuresRobot || subgraphRobot)
  if (inClosuresRobot || cemsg)
    return gmsg;
  else
    return 0;

}

GraphMessage* MRGraphSLAM::constructGraphMessage(int idRobotTo){
  boost::mutex::scoped_lock lockg(graphMutex);

  RobotMessage* msg = factory->constructMessage(8);
  GraphMessage* gmsg = dynamic_cast<GraphMessage*>(msg);
  gmsg->setRobotId(idRobot());

  OptimizableGraph::VertexIDMap* inClosuresRobot = condensedGraphs.inClosures(idRobotTo);
  if (inClosuresRobot){
    gmsg->closures.resize(inClosuresRobot->size());
    int i = 0;
    for (OptimizableGraph::VertexIDMap::iterator it = inClosuresRobot->begin(); it != inClosuresRobot->end(); it++){
      int vId = it->first;
      //cerr << "Asking for node " << vId << endl;
      gmsg->closures[i] = vId;
      i++;
    }
  }

  OptimizableGraph::VertexIDMap* outClosuresRobot = condensedGraphs.outClosures(idRobotTo);
  if (outClosuresRobot){
    //Robot has asked for some nodes
    //We send the whole graph

    EdgeArrayMessage* emsg = dynamic_cast<EdgeArrayMessage*>(gmsg);
    OptimizableGraph::EdgeSet myOwnEdges = graph()->edges();
    for (std::map<int, OptimizableGraph::EdgeSet>::iterator it = condensedGraphs.inCondensedGraphs().begin(); it != condensedGraphs.inCondensedGraphs().end(); it++){
      OptimizableGraph::EdgeSet edgesrobot = it->second;
      for (OptimizableGraph::EdgeSet::iterator ite = edgesrobot.begin(); ite != edgesrobot.end(); ite++){
	//Remove edge from other robot
	OptimizableGraph::Edge* edge = (OptimizableGraph::Edge*) (*ite);
	myOwnEdges.erase(edge);
      }
    }
    for (std::map<int, OptimizableGraph::EdgeSet>::iterator it = condensedGraphs.outCondensedGraphs().begin(); it != condensedGraphs.outCondensedGraphs().end(); it++){
      OptimizableGraph::EdgeSet edgesrobot = it->second;
      for (OptimizableGraph::EdgeSet::iterator ite =  edgesrobot.begin(); ite != edgesrobot.end(); ite++){
	OptimizableGraph::Edge* edge = (OptimizableGraph::Edge*) (*ite);
	myOwnEdges.erase(edge);
      }
    }

    
    EdgeArrayMessage* cemsg = constructEdgeArrayMessage(myOwnEdges);

    if (cemsg)
      *emsg = *cemsg;
    
    OptimizableGraph::VertexIDMap myOwnVertices;
    for (OptimizableGraph::VertexIDMap::iterator it = graph()->vertices().begin(); it != graph()->vertices().end(); it++){
      OptimizableGraph::Vertex *v = (OptimizableGraph::Vertex *)it->second;
      if (isMyVertex(v)){
	myOwnVertices.insert(*it);
      }
    }
    VertexArrayMessage* vmsg = dynamic_cast<VertexArrayMessage*>(gmsg);
    VertexArrayMessage* cvmsg = constructVertexArrayMessage(myOwnVertices);

    if (cvmsg)
      *vmsg = *cvmsg;
  }

  if (inClosuresRobot || outClosuresRobot)
    return gmsg;
  else
    return 0;

}

RobotMessage* MRGraphSLAM::createMsgfromCharArray(const char* buffer, size_t size){
  boost::mutex::scoped_lock lockg(graphMutex);

  RobotMessage* msg = factory->fromCharArray(buffer, size);
  return msg;
}
