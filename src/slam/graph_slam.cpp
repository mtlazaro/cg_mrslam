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

#include "graph_slam.h"
#include "graph_manipulator.h"

GraphSLAM::GraphSLAM():_vf(0){ 
  _firstRobotPose = 0; 
  _lastVertex = 0;  
  _idRobot = 0; 
  _baseId = 10000; 
  _runningVertexId = 0; 
  _runningEdgeId = 0;
  _graph = new SparseOptimizer();
  _vf = VerticesFinder(_graph);
}


typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

void GraphSLAM::init(double resolution, double kernelRadius, int windowLoopClosure_, double maxScore_, double inlierThreshold_, int minInliers_){
  boost::mutex::scoped_lock lockg(graphMutex);

  //Init graph
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(blockSolver);
  _graph->setAlgorithm(solver);
  _graph->setVerbose(false);

  //Init scan matchers
  _closeMatcher.initializeKernel(resolution, kernelRadius);
  _closeMatcher.initializeGrid(Vector2f(-15, -15), Vector2f(15, 15), resolution);
  _LCMatcher.initializeKernel(0.1, 0.5); //before 0.1, 0.5
  _LCMatcher.initializeGrid(Vector2f(-35, -35), Vector2f(35, 35), 0.1); //before 0.1
  cerr << "Grids initialized\n";


  windowLoopClosure = windowLoopClosure_;
  maxScore = maxScore_;
  inlierThreshold = inlierThreshold_;
  minInliers = minInliers_;

  //
  _odominf =  100 * Matrix3d::Identity();
  _odominf(2,2) = 1000;

  _SMinf = 1000 * Matrix3d::Identity();
  _SMinf(2,2) = 10000;
}

void GraphSLAM::setIdRobot(int idRobot){
  _idRobot = idRobot;
}

void GraphSLAM::setBaseId(int baseId){
  _baseId = baseId;
}

void GraphSLAM::setInitialData(SE2 initialTruePose, SE2 initialOdom, RobotLaser* laser){
  boost::mutex::scoped_lock lockg(graphMutex);

  _lastOdom = initialOdom;
  //Add initial vertex
  _lastVertex = new VertexSE2;

  _lastVertex->setEstimate(initialTruePose);
  _lastVertex->setId(idRobot() * baseId());

  //Add covariance information
  //VertexEllipse *ellipse = new VertexEllipse();
  //Matrix3f cov = Matrix3f::Zero(); //last vertex has zero covariance
  //ellipse->setCovariance(cov);
  //_lastVertex->setUserData(ellipse);
  _lastVertex->setUserData(laser);
  
  std::cout << 
    "Initial vertex: " << _lastVertex->id() << 
    " Estimate: "<< _lastVertex->estimate().translation().x() << 
    " " << _lastVertex->estimate().translation().y() << 
    " " << _lastVertex->estimate().rotation().angle() << std::endl;

  _graph->addVertex(_lastVertex);

  _firstRobotPose = _lastVertex;
  _firstRobotPose->setFixed(true);
}

void GraphSLAM::setInitialData(SE2 initialOdom, RobotLaser* laser){
  boost::mutex::scoped_lock lockg(graphMutex);

  _lastOdom = initialOdom;
  //Add initial vertex
  _lastVertex = new VertexSE2;

  _lastVertex->setEstimate(_lastOdom);
  _lastVertex->setId(idRobot() * baseId());
  
  //Add covariance information
  //VertexEllipse *ellipse = new VertexEllipse();
  //Matrix3f cov = Matrix3f::Zero(); //last vertex has zero covariance
  //ellipse->setCovariance(cov);
  //_lastVertex->setUserData(ellipse);
  _lastVertex->addUserData(laser);

  std::cout << 
    "Initial vertex: " << _lastVertex->id() << 
    " Estimate: "<< _lastVertex->estimate().translation().x() << 
    " " << _lastVertex->estimate().translation().y() << 
    " " << _lastVertex->estimate().rotation().angle() << std::endl;

  _graph->addVertex(_lastVertex);

  _firstRobotPose = _lastVertex;
  _firstRobotPose->setFixed(true);
}

void GraphSLAM::addData(SE2 currentOdom, RobotLaser* laser){
  boost::mutex::scoped_lock lockg(graphMutex);

  //Add current vertex
  VertexSE2 *v = new VertexSE2;

  SE2 displacement = _lastOdom.inverse() * currentOdom;
  SE2 currEst = _lastVertex->estimate() * displacement;

  v->setEstimate(currEst);
  v->setId(++_runningVertexId + idRobot() * baseId());
  //Add covariance information
  //VertexEllipse *ellipse = new VertexEllipse;
  //Matrix3f cov = Matrix3f::Zero(); //last vertex has zero covariance
  //ellipse->setCovariance(cov);
  //v->setUserData(ellipse);
  v->addUserData(laser);

  std::cout << std::endl << 
    "Current vertex: " << v->id() << 
    " Estimate: "<< v->estimate().translation().x() << 
    " " << v->estimate().translation().y() << 
    " " << v->estimate().rotation().angle() << std::endl;

  _graph->addVertex(v);

  //Add current odometry edge
  EdgeSE2 *e = new EdgeSE2;
  e->setId(++_runningEdgeId + idRobot() * baseId());
  e->vertices()[0] = _lastVertex;
  e->vertices()[1] = v;
      
  e->setMeasurement(displacement);
  
  // //Computing covariances depending on the displacement
  // Vector3d dis = displacement.toVector();
  // dis.x() = fabs(dis.x());
  // dis.y() = fabs(dis.y());
  // dis.z() = fabs(dis.z());
  // dis += Vector3d(1e-3,1e-3,1e-2);  
  // Matrix3d dis2 = dis*dis.transpose();
  // Matrix3d newcov = dis2.cwiseProduct(_odomK);

  e->setInformation(_odominf);
  _graph->addEdge(e);

  _odomEdges.insert(e);

  _lastOdom = currentOdom;
  _lastVertex = v;
}

void GraphSLAM::addDataSM(SE2 currentOdom, RobotLaser* laser){
  boost::mutex::scoped_lock lockg(graphMutex);

  //Add current vertex
  VertexSE2 *v = new VertexSE2;

  SE2 displacement = _lastOdom.inverse() * currentOdom;
  SE2 currEst = _lastVertex->estimate() * displacement;

  v->setEstimate(currEst);
  v->setId(++_runningVertexId + idRobot() * baseId());
  //Add covariance information
  //VertexEllipse *ellipse = new VertexEllipse;
  //Matrix3f cov = Matrix3f::Zero(); //last vertex has zero covariance
  //ellipse->setCovariance(cov);
  //v->setUserData(ellipse);
  v->addUserData(laser);

  std::cout << endl << 
    "Current vertex: " << v->id() << 
    " Estimate: "<< v->estimate().translation().x() << 
    " " << v->estimate().translation().y() << 
    " " << v->estimate().rotation().angle() << std::endl;

  _graph->addVertex(v);

  //Add current odometry edge
  EdgeSE2 *e = new EdgeSE2;
  e->setId(++_runningEdgeId + idRobot() * baseId());
  e->vertices()[0] = _lastVertex;
  e->vertices()[1] = v;
      

  OptimizableGraph::VertexSet vset;
  vset.insert(_lastVertex);
  int j = 1;
  int gap = 5;
  while (j <= gap){
    VertexSE2 *vj =  dynamic_cast<VertexSE2 *>(graph()->vertex(_lastVertex->id()-j));
    if (vj)
      vset.insert(vj);
    else
      break;
    j++;
  }

  SE2 transf;
  bool shouldIAdd = _closeMatcher.closeScanMatching(vset, _lastVertex, v,  &transf, maxScore);

  if (shouldIAdd){
    e->setMeasurement(transf);
    e->setInformation(_SMinf);
  }else{ //Trust the odometry
    e->setMeasurement(displacement);
    // Vector3d dis = displacement.toVector();
    // dis.x() = fabs(dis.x());
    // dis.y() = fabs(dis.y());
    // dis.z() = fabs(dis.z());
    // dis += Vector3d(1e-3,1e-3,1e-2);  
    // Matrix3d dis2 = dis*dis.transpose();
    // Matrix3d newcov = dis2.cwiseProduct(_odomK);
    // e->setInformation(newcov.inverse());

    e->setInformation(_odominf);
  }

  _graph->addEdge(e);

  _lastOdom = currentOdom;
  _lastVertex = v;
}



RobotLaser* GraphSLAM::findLaserData(OptimizableGraph::Vertex* v){
  HyperGraph::Data* d = v->userData();

  while (d){
    RobotLaser* robotLaser = dynamic_cast<RobotLaser*>(d);
    if (robotLaser){
      return robotLaser;
    }else{
      d = d->next();
    }
  }

  return 0;
}
/*
VertexEllipse* GraphSLAM::findEllipseData(OptimizableGraph::Vertex* v){
  HyperGraph::Data* d = v->userData();

  while (d){
    VertexEllipse* ellipse = dynamic_cast<VertexEllipse*>(d);
    if (ellipse){
      return ellipse;
    }else{
      d = d->next();
    }
  }

  return 0;
}*/

void GraphSLAM::checkHaveLaser(OptimizableGraph::VertexSet& vset){
  OptimizableGraph::VertexSet tmpvset = vset;
  for (OptimizableGraph::VertexSet::iterator it = tmpvset.begin(); it != tmpvset.end(); it++){
    OptimizableGraph::Vertex *vertex = (OptimizableGraph::Vertex*) *it;
    if (!findLaserData(vertex))
      vset.erase(*it);
  }
}


void GraphSLAM::checkCovariance(OptimizableGraph::VertexSet& vset){
  ///////////////////////////////////
  // we need now to compute the marginal covariances of all other vertices w.r.t the newly inserted one

  CovarianceEstimator ce(_graph);

  ce.setVertices(vset);
  ce.setGauge(_lastVertex);
  
  ce.compute();

  assert(!_lastVertex->fixed() && "last Vertex is fixed");
  assert(_firstRobotPose->fixed() && "first Vertex is not fixed");
  
  OptimizableGraph::VertexSet tmpvset = vset;
  for (OptimizableGraph::VertexSet::iterator it = tmpvset.begin(); it != tmpvset.end(); it++){
    VertexSE2 *vertex = (VertexSE2*) *it;
    
    MatrixXd Pv = ce.getCovariance(vertex);
    Matrix2d Pxy; Pxy << Pv(0,0), Pv(0,1), Pv(1,0), Pv(1,1);
    SE2 delta = vertex->estimate().inverse() * _lastVertex->estimate();	
    Vector2d hxy (delta.translation().x(), delta.translation().y());
    double perceptionRange =1;
    if (hxy.x()-perceptionRange>0) 
      hxy.x() -= perceptionRange;
    else if (hxy.x()+perceptionRange<0)
      hxy.x() += perceptionRange;
    else
      hxy.x() = 0;

    if (hxy.y()-perceptionRange>0) 
      hxy.y() -= perceptionRange;
    else if (hxy.y()+perceptionRange<0)
      hxy.y() += perceptionRange;
    else
      hxy.y() = 0;
    
    double d2 = hxy.transpose() * Pxy.inverse() * hxy;
    if (d2 > 5.99)
      vset.erase(*it);
 
  }
  
}

void GraphSLAM::addNeighboringVertices(OptimizableGraph::VertexSet& vset, int gap){
  OptimizableGraph::VertexSet temp = vset;
  for (OptimizableGraph::VertexSet::iterator it = temp.begin(); it!=temp.end(); it++){
    OptimizableGraph::Vertex* vertex = (OptimizableGraph::Vertex*) *it;
    for (int i = 1; i <= gap; i++){
      OptimizableGraph::Vertex *v = (OptimizableGraph::Vertex *) _graph->vertex(vertex->id()+i);
      if (v && v->id() != _lastVertex->id()){
	OptimizableGraph::VertexSet::iterator itv = vset.find(v);
	if (itv == vset.end())
	  vset.insert(v);
	else
	  break;
      }
    }

    for (int i = 1; i <= gap; i++){
      OptimizableGraph::Vertex* v = (OptimizableGraph::Vertex*) _graph->vertex(vertex->id()-i);
      if (v && v->id() != _lastVertex->id()){
	OptimizableGraph::VertexSet::iterator itv = vset.find(v);
	if (itv == vset.end())
	  vset.insert(v);
	else
	  break;
      }
    }
  }
}

bool GraphSLAM::isMyVertex(OptimizableGraph::Vertex *v){
  return (v->id()/baseId()) == idRobot();
}

void GraphSLAM::findConstraints(){
  boost::mutex::scoped_lock lockg(graphMutex);

  OptimizableGraph::VertexSet vset;
  _vf.findVerticesScanMatching( _lastVertex, vset);

  checkCovariance(vset);
  addNeighboringVertices(vset, 8);
  checkHaveLaser(vset);

  std::set<OptimizableGraph::VertexSet> setOfVSet;
  _vf.findSetsOfVertices(vset, setOfVSet);
      
 
  OptimizableGraph::EdgeSet loopClosingEdges;
  for (std::set<OptimizableGraph::VertexSet>::iterator it = setOfVSet.begin(); it != setOfVSet.end(); it++) {
    
    OptimizableGraph::VertexSet myvset = *it;
    
    OptimizableGraph::Vertex* closestV = _vf.findClosestVertex(myvset, _lastVertex); 
    
    if (closestV->id() == _lastVertex->id() - 1) //Already have this edge
      continue;

    SE2 transf;
    if (!isMyVertex(closestV) || (isMyVertex(closestV) && abs(_lastVertex->id() - closestV->id()) > 10)){
      /*VertexEllipse* ellipse = findEllipseData(_lastVertex);
      if (ellipse){
	for (OptimizableGraph::VertexSet::iterator itv = myvset.begin(); itv != myvset.end(); itv++){
	  VertexSE2 *vertex = (VertexSE2*) *itv;
	  SE2 relativetransf = _lastVertex->estimate().inverse() * vertex->estimate();
	  ellipse->addMatchingVertex(relativetransf.translation().x(), relativetransf.translation().y());
	  ellipse->addMatchingVertexID(vertex->id());
	}
      }*/

      std::vector<SE2> results;

      /*OptimizableGraph::VertexSet referenceVset;
	referenceVset.insert(_lastVertex);
	int j = 1;
	int comm_gap = 5;
	while (j <= comm_gap){
	VertexSE2 *vj =  dynamic_cast<VertexSE2 *>(graph()->vertex(_lastVertex->id()-j));
	if (vj)
	referenceVset.insert(vj);
	else
	break;
	j++;
	}*/

      //Loop Closing Edge
      bool shouldIAdd = _LCMatcher.scanMatchingLC(myvset,  closestV, _lastVertex,  results, maxScore);
      //bool shouldIAdd = _mf.scanMatchingLC(myvset,  closestV, referenceVset, _lastVertex,  results, maxScore);
      if (shouldIAdd){
	for (unsigned int i =0; i< results.size(); i++){
	  EdgeSE2 *ne = new EdgeSE2;
	  ne->setId(++_runningEdgeId + _baseId);
	  ne->vertices()[0] = closestV;
	  ne->vertices()[1] = _lastVertex;
	  ne->setMeasurement(results[i]);
	  ne->setInformation(_SMinf);
	
	  loopClosingEdges.insert(ne);
	  _SMEdges.insert(ne);
	}
      }else {
	std::cout << "Rejecting LC edge between " << closestV->id() << " and " << _lastVertex->id() << " [matching fail] " << std::endl;
      }
    }else{
      //Edge between close vertices
      bool shouldIAdd = _closeMatcher.closeScanMatching(myvset, closestV, _lastVertex, &transf, maxScore);
      if (shouldIAdd){
	EdgeSE2 *ne = new EdgeSE2;
	ne->setId(++_runningEdgeId + _baseId);
	ne->vertices()[0] = closestV;
	ne->vertices()[1] = _lastVertex;
	ne->setMeasurement(transf);
	ne->setInformation(_SMinf);
	
	_graph->addEdge(ne);
	_SMEdges.insert(ne);
      }else {
	std::cout << "Rejecting edge between " << closestV->id() << " and " << _lastVertex->id() << " [matching fail] " << std::endl;
      }
    }
  }
  
  if (loopClosingEdges.size())
    addClosures(loopClosingEdges);
  
  checkClosures();
  updateClosures();
}

void GraphSLAM::addClosures(OptimizableGraph::EdgeSet loopClosingEdges){

  _closures.addEdgeSet(loopClosingEdges);
  _closures.addVertex(_lastVertex);
}

void GraphSLAM::checkClosures(){
  if (_closures.checkList(windowLoopClosure)){
    cout << endl << "Loop Closure Checking." << endl;
    // for(std::list<VertexTime>::iterator it = _closures.vertexList().begin(); it!= _closures.vertexList().end(); it++){
    //   VertexTime vt = *it;
    //   cout << "In list: Vertex: "  << vt.v->id() << " time: " << vt.time << endl;
    // }
    
    lcc.init(_closures.vertices(), _closures.edgeSet(), inlierThreshold);
    lcc.check();

    // for (LoopClosureChecker::EdgeDoubleMap::iterator it = lcc.closures().begin(); it!= lcc.closures().end(); it++){
    //   EdgeSE2* e = (EdgeSE2*) (it->first);
    //   VertexSE2* vfrom=dynamic_cast<VertexSE2*>(e->vertices()[0]);
    //   VertexSE2* vto=dynamic_cast<VertexSE2*>(e->vertices()[1]);
      
    //   cerr << "Edge from: " << vfrom->id() << " " << vto->id() 
    // 	   << " Estimate: " << e->measurement().translation().x() << " " << e->measurement().translation().y() << " " << e->measurement().rotation().angle() 
    // 	   << " Chi2 = " << it->second << endl;
      
    // }


    cout << "Best Chi2 = " << lcc.chi2() << endl;
    cout << "Inliers = " << lcc.inliers() << endl;

    if (lcc.inliers() >= minInliers){
      LoopClosureChecker::EdgeDoubleMap results = lcc.closures();
      cout << "Results:" << endl;
      for (LoopClosureChecker::EdgeDoubleMap::iterator it= results.begin(); it!= results.end(); it++){
	EdgeSE2* e = (EdgeSE2*) (it->first);
	cout << "Edge from: " << e->vertices()[0]->id() << " to: " << e->vertices()[1]->id() << ". Chi2 = " << it->second <<  endl;

	if (it->second < inlierThreshold){
	  cout << "Is an inlier. Adding to Graph" << endl;
	  _graph->addEdge(e);
	}
      }
    }
  }
}


void GraphSLAM::updateClosures(){

  _closures.updateList(windowLoopClosure);
  
  // for(std::list<VertexTime>::iterator it = _closures.vertexList().begin(); it!=_closures.vertexList().end(); it++){
  //   VertexTime vt = (*it);
  //   std::cout << "Vertex in list: " << vt.v->id() << " Time: " << vt.time << std::endl;
  // }
  // for (OptimizableGraph::VertexIDMap::iterator it = _closures.vertices().begin(); it != _closures.vertices().end(); it++){
  //   VertexSE2* vertex= (VertexSE2*)(it->second);
  //   std::cout << "Vertex in closures after update: " << vertex->id() << " Estimate: " << vertex->estimate().translation().x() << " " 
  // 	      << vertex->estimate().translation().y() << " "
  // 	      << vertex->estimate().rotation().angle() << std::endl; 
  // }
  // for (OptimizableGraph::EdgeSet::iterator it = _closures.edgeSet().begin(); it!=_closures.edgeSet().end(); it++){
  //   EdgeSE2* e=(EdgeSE2*) (*it);
  //   VertexSE2* vfrom=dynamic_cast<VertexSE2*>(e->vertices()[0]);
  //   VertexSE2* vto=dynamic_cast<VertexSE2*>(e->vertices()[1]);
  //   std::cout << "Edge in closures after: From: " << vfrom->id() << " to: " << vto->id() << " Measurement: [" 
  // 	      << e->measurement().translation().x() << " " 
  // 	      << e->measurement().translation().y() << " " 
  // 	      << e->measurement().rotation().angle() << "]" << std::endl;
  // }
}

void GraphSLAM::optimize(int nrunnings){
  boost::mutex::scoped_lock lockg(graphMutex);

  _graph->initializeOptimization();
  _graph->optimize(nrunnings);

  //////////////////////////////////////
  //Update laser data
  for (SparseOptimizer::VertexIDMap::const_iterator it = _graph->vertices().begin(); it != _graph->vertices().end(); ++it) {
    VertexSE2* v = dynamic_cast<VertexSE2*>(it->second);
    RobotLaser* robotLaser = findLaserData(v);
    if (robotLaser) 
      robotLaser->setOdomPose(v->estimate());
  }
  
  //////////////////////////////////////
  //Update ellipse data
  //Covariance computation
  /*
  CovarianceEstimator ce(_graph);

  OptimizableGraph::VertexSet vset;
  for (OptimizableGraph::VertexIDMap::iterator it=_graph->vertices().begin(); it!=_graph->vertices().end(); ++it) {
    OptimizableGraph::Vertex* v = (OptimizableGraph::Vertex*) (it->second);
    vset.insert(v);
  }

  ce.setVertices(vset);
  ce.setGauge(_lastVertex);

  ce.compute();

  ///////////////////////////////////
  
  for (OptimizableGraph::VertexIDMap::iterator it=_graph->vertices().begin(); it!=_graph->vertices().end(); ++it) {
    VertexSE2* v = dynamic_cast<VertexSE2*>(it->second);
    VertexEllipse* ellipse = findEllipseData(v);
    if (ellipse && (v != lastVertex())){
      MatrixXd PvX = ce.getCovariance(v);
      Matrix3d Pv = PvX;
      Matrix3f Pvf = Pv.cast<float>();
      ellipse->setCovariance(Pvf);
      ellipse->clearMatchingVertices();
    }else {
      if(ellipse && v == lastVertex()){
	ellipse->clearMatchingVertices();
	for (size_t i = 0; i<ellipse->matchingVerticesIDs().size(); i++){
	  int id = ellipse->matchingVerticesIDs()[i];
	  VertexSE2* vid = dynamic_cast<VertexSE2*>(_graph->vertex(id));
	  SE2 relativetransf = _lastVertex->estimate().inverse() * vid->estimate();
	  ellipse->addMatchingVertex(relativetransf.translation().x(), relativetransf.translation().y());
	}
      }
    }
  }*/
  
}


bool GraphSLAM::saveGraph(const char *filename){
  boost::mutex::scoped_lock lockg(graphMutex);
  return _graph->save(filename);
}

bool GraphSLAM::loadGraph(const char *filename){
  boost::mutex::scoped_lock lockg(graphMutex);
  return _graph->load(filename);
}
