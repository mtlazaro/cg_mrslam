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

#include <limits>
#include "scan_matcher.h"
#include <iostream>
using namespace std;

ScanMatcher::ScanMatcher() :_grid(Eigen::Vector2f(-15, -15), Eigen::Vector2f(15, 15), 0.025, 128){
 _kscale =  128;
 }

void ScanMatcher::initializeKernel(double resolution, double kernelRange){
  int size = kernelRange/resolution;
  int center = size;
  int dim = 2*size+1;
    
  _kernel.resize(dim,dim);

  int K1=resolution * _kscale;
  int K2=kernelRange * _kscale;
  _kernel.fill((char) K2);

  for(int j = 0; j <= size; j++){
    for(int i = 0; i <= size; i++){
      char distance = K1*sqrt(j*j+i*i);
      if(distance > K2)
	continue;
      _kernel(i+center,j+center) = distance;
      _kernel(center-i,j+center) = distance;
      _kernel(i+center,center-j) = distance;
      _kernel(center-i,center-j) = distance;
    }
  }
  _kernelRange = kernelRange;
}

void ScanMatcher::initializeGrid(Eigen::Vector2f lowerLeft, Eigen::Vector2f upperRight, double resolution){
  CharGrid tmp(lowerLeft, upperRight, resolution, _kscale);
  _grid = tmp;
}

void ScanMatcher::resetGrid() {
  int K2=_kernelRange * _kscale;
  
  Eigen::Vector2i size = _grid.grid().size();
  for (int i = 0; i<size.x(); i++)
    for (int j = 0; j<size.y(); j++)
      _grid.grid().cell(i,j) = K2;

}
 
void ScanMatcher::applyTransfToScan(SE2 transf, RawLaser::Point2DVector scan, RawLaser::Point2DVector& outScan){
  outScan.resize(scan.size());
  for (unsigned int i = 0; i < scan.size(); i++){
    SE2 point;
    point.setTranslation(scan[i]);
    
    SE2 rp = transf * point;
    outScan[i] = rp.translation();
  }
}

void transformPointsFromVSet(OptimizableGraph::VertexSet& vset, OptimizableGraph::Vertex* _referenceVertex, RawLaser::Point2DVector& scansInRefVertex){

  VertexSE2* referenceVertex=dynamic_cast<VertexSE2*>(_referenceVertex);
  scansInRefVertex.clear();
  for (OptimizableGraph::VertexSet::iterator it = vset.begin(); it != vset.end(); it++){
    VertexSE2 *vertex = (VertexSE2*) *it;
    RobotLaser* laserv = dynamic_cast<RobotLaser*>(vertex->userData());
    if (laserv){
      RawLaser::Point2DVector vscan = laserv->cartesian();
      SE2 trl = laserv->laserParams().laserPose;
      RawLaser::Point2DVector scanInRefVertex;
      if (vertex->id() == referenceVertex->id()){
	ScanMatcher::applyTransfToScan(trl, vscan, scanInRefVertex);
      }else{
	SE2 trel = referenceVertex->estimate().inverse() * vertex->estimate();
	SE2 transf = trel * trl;
	ScanMatcher::applyTransfToScan(transf, vscan, scanInRefVertex);
      }
      scansInRefVertex.insert(scansInRefVertex.end(), scanInRefVertex.begin(), scanInRefVertex.end());
    }
  }
}

bool ScanMatcher::closeScanMatching(OptimizableGraph::VertexSet& vset, OptimizableGraph::Vertex* _originVertex, OptimizableGraph::Vertex* _currentVertex,  SE2 *trel, double maxScore){ 
  
  VertexSE2* currentVertex=dynamic_cast<VertexSE2*>(_currentVertex);
  VertexSE2* originVertex =dynamic_cast<VertexSE2*>(_originVertex);

  resetGrid();
  
  RawLaser::Point2DVector scansInRefVertex;
  transformPointsFromVSet(vset, _originVertex, scansInRefVertex);
  _grid.addAndConvolvePoints<RawLaser::Point2DVector>(scansInRefVertex.begin(), scansInRefVertex.end(), _kernel);

  //Current vertex
  RobotLaser* lasercv = dynamic_cast<RobotLaser*>(currentVertex->userData());
  
  if (!lasercv)
    return false;

  RawLaser::Point2DVector cvscan = lasercv->cartesian();
  SE2 laserPoseCV = lasercv->laserParams().laserPose;
  RawLaser::Point2DVector cvScanRobot;
  applyTransfToScan(laserPoseCV, cvscan, cvScanRobot);

  SE2 delta = originVertex->estimate().inverse() * currentVertex->estimate();

  Eigen::Vector3d initGuess(delta.translation().x(), delta.translation().y(), delta.rotation().angle());

  std::vector<MatcherResult> mresvec;
  clock_t t_ini, t_fin;
  double secs;

  t_ini = clock();

  double thetaRes = 0.0125*.5; // was 0.01
  Eigen::Vector3f lower(-.3+initGuess.x(), -.3+initGuess.y(), -0.2+initGuess.z());
  Eigen::Vector3f upper(+.3+initGuess.x(),  .3+initGuess.y(),  0.2+initGuess.z()); 
  _grid.greedySearch(mresvec, cvScanRobot, lower, upper, thetaRes, maxScore, 0.5, 0.5, 0.2);
  t_fin = clock();

  secs = (double)(t_fin - t_ini) / CLOCKS_PER_SEC;
  printf("Greedy search: %.16g ms. Matcher results: %i\n", secs * 1000.0, (int) mresvec.size());

  if (mresvec.size()){
    Eigen::Vector3d adj=mresvec[0].transformation;
    trel->setTranslation(Eigen::Vector2d(adj.x(), adj.y()));
    trel->setRotation(adj.z());
    //cerr <<  "bestScore = " << mresvec[0].score << endl << endl; 

    // if (currentVertex->id() > 120 && currentVertex->id() < 200){
    //   CharMatcher auxGrid = _grid;

    //   Vector2dVector transformedScan;
    //   transformedScan.resize(cvScanRobot.size());
    //   for (unsigned int i = 0; i<cvScanRobot.size(); i++){
    // 	SE2 point;
    // 	point.setTranslation(cvScanRobot[i]);
      
    // 	SE2 transformedpoint = *trel * point;
    // 	transformedScan[i] = transformedpoint.translation();
    //   }
    //   auxGrid.addPoints<RawLaser::Point2DVector>(transformedScan.begin(), transformedScan.end());

    //   ofstream image;
    //   std::stringstream filename;
    //   filename << "matchedmap" << currentVertex->id() << "_" << mresvec[0].score << ".ppm";
    //   image.open(filename.str().c_str());
    //   auxGrid.grid().saveAsPPM(image, false);
    // }

    return true;
  } 
  cerr << endl;
  return false;

}

bool ScanMatcher::scanMatchingLC(OptimizableGraph::VertexSet& referenceVset,  OptimizableGraph::Vertex* _referenceVertex, OptimizableGraph::Vertex* _currentVertex,  std::vector<SE2>& trel, double maxScore){ 

  OptimizableGraph::VertexSet  currvset;
  currvset.insert(_currentVertex);

  return scanMatchingLC(referenceVset, _referenceVertex, currvset, _currentVertex, trel, maxScore);
  //return scanMatchingLChierarchical(referenceVset, _originVertex, currvset, _currentVertex, trel, maxScore);
}


bool ScanMatcher::scanMatchingLC(OptimizableGraph::VertexSet& referenceVset,  OptimizableGraph::Vertex* _referenceVertex, OptimizableGraph::VertexSet& currvset, OptimizableGraph::Vertex* _currentVertex,  std::vector<SE2>& trel, double maxScore){ 
  cerr << "Loop Closing Scan Matching" << endl;
  //cerr << "Size of Vset " << referenceVset.size() << endl;
  VertexSE2* referenceVertex =dynamic_cast<VertexSE2*>(_referenceVertex);

  resetGrid();
  trel.clear();
  
  RawLaser::Point2DVector scansInRefVertex;
  transformPointsFromVSet(referenceVset, _referenceVertex, scansInRefVertex);
  _grid.addAndConvolvePoints<RawLaser::Point2DVector>(scansInRefVertex.begin(), scansInRefVertex.end(), _kernel);

  RawLaser::Point2DVector scansInCurVertex;
  transformPointsFromVSet(currvset, _currentVertex, scansInCurVertex);

  Vector2dVector reducedScans;
  CharGrid::subsample(reducedScans, scansInCurVertex, 0.1);

  RegionVector regions;
  RegionVector regionspi;
  for (OptimizableGraph::VertexSet::iterator it = referenceVset.begin(); it != referenceVset.end(); it++){
    VertexSE2 *vertex = (VertexSE2*) *it;

    Region reg;
    SE2 relposv(.0, .0, .0);
    if (vertex->id() != referenceVertex->id())
      relposv = referenceVertex->estimate().inverse() * vertex->estimate();
    
    Eigen::Vector3f lower(-.5+relposv.translation().x(), -2.+relposv.translation().y(), -1.+relposv.rotation().angle());
    Eigen::Vector3f upper( .5+relposv.translation().x(),  2.+relposv.translation().y(),  1.+relposv.rotation().angle());
    reg.lowerLeft  = lower;
    reg.upperRight = upper;
    regions.push_back(reg);
  
    lower[2] += M_PI;
    upper[2] += M_PI; 
    reg.lowerLeft  = lower;
    reg.upperRight = upper;
    regionspi.push_back(reg);
  }

  std::vector<MatcherResult> mresvec;
  double thetaRes = 0.025; // was 0.0125*.5
  //Results discretization
  double dx = 0.5, dy = 0.5, dth = 0.2;
  
  std::map<DiscreteTriplet, MatcherResult> resultsMap;
  
  clock_t t_ini, t_fin;
  double secs;
  
  t_ini = clock();
  _grid.greedySearch(mresvec, reducedScans, regions, thetaRes, maxScore, dx, dy, dth);
  t_fin = clock();
  secs = (double)(t_fin - t_ini) / CLOCKS_PER_SEC;
  printf("%.16g ms. Matcher results: %i\n", secs * 1000.0, (int) mresvec.size());
  
  if (mresvec.size()){
    mresvec[0].transformation[2] = normalize_theta(mresvec[0].transformation[2]);
    cerr << "Found Loop Closure Edge. Transf: " << mresvec[0].transformation.x() << " " << mresvec[0].transformation.y() << " " << mresvec[0].transformation.z() << endl;

    CharGrid::addToPrunedMap(resultsMap, mresvec[0], dx, dy, dth);
  }

  t_ini = clock();
  _grid.greedySearch(mresvec, reducedScans, regionspi, thetaRes, maxScore, dx, dy, dth);
  t_fin = clock();
  secs = (double)(t_fin - t_ini) / CLOCKS_PER_SEC;
  printf("%.16g ms. Matcher results: %i\n", secs * 1000.0, (int) mresvec.size());

  if (mresvec.size()){
    mresvec[0].transformation[2] = normalize_theta(mresvec[0].transformation[2]);
    cerr << "Found Loop Closure Edge PI. Transf: " << mresvec[0].transformation.x() << " " << mresvec[0].transformation.y() << " " << mresvec[0].transformation.z() << endl;

    CharGrid::addToPrunedMap(resultsMap, mresvec[0], dx, dy, dth);
  }

  for (std::map<DiscreteTriplet, MatcherResult>::iterator it = resultsMap.begin(); it!= resultsMap.end(); it++){
    MatcherResult res = it->second;
    Eigen::Vector3d adj=res.transformation;
    SE2 transf;
    transf.setTranslation(Eigen::Vector2d(adj.x(), adj.y()));
    transf.setRotation(normalize_theta(adj.z()));
    trel.push_back(transf);
    
    std::cerr << "Final result: " << transf.translation().x() << " " << transf.translation().y() << " " << transf.rotation().angle() << std::endl;
  }

  if (trel.size())
    return true;

  return false;
}

bool ScanMatcher::scanMatchingLChierarchical(OptimizableGraph::VertexSet& referenceVset,  OptimizableGraph::Vertex* _referenceVertex, OptimizableGraph::VertexSet& currvset, OptimizableGraph::Vertex* _currentVertex,  std::vector<SE2>& trel, double maxScore){ 
  //cerr << "Loop Closing Scan Matching" << endl;
  //cerr << "Size of Vset " << referenceVset.size() << endl;
  VertexSE2* currentVertex=dynamic_cast<VertexSE2*>(_currentVertex);
  VertexSE2* referenceVertex =dynamic_cast<VertexSE2*>(_referenceVertex);

  resetGrid();
  trel.clear();

  RawLaser::Point2DVector scansInRefVertex;
  transformPointsFromVSet(referenceVset, _referenceVertex, scansInRefVertex);
  _grid.addAndConvolvePoints<RawLaser::Point2DVector>(scansInRefVertex.begin(), scansInRefVertex.end(), _kernel);
  
  RawLaser::Point2DVector scansInCurVertex;
  transformPointsFromVSet(currvset, _currentVertex, scansInCurVertex);

  Vector2dVector reducedScans;
  CharGrid::subsample(reducedScans, scansInCurVertex, 0.1);
  //cerr << "subsampling: " << scansInCurVertex.size() << " -> " << reducedScans.size() << endl;

  SE2 delta = referenceVertex->estimate().inverse() * currentVertex->estimate();

  Eigen::Vector3d initGuess(delta.translation().x(), delta.translation().y(), delta.rotation().angle());

  Eigen::Vector3f lower(-2.+initGuess.x(), -2.+initGuess.y(), -1.+initGuess.z());
  Eigen::Vector3f upper(+2.+initGuess.x(),  2.+initGuess.y(),  1.+initGuess.z()); 

  RegionVector regions;
  Region reg;
  reg.lowerLeft  = lower;
  reg.upperRight = upper;
  regions.push_back(reg);

  std::vector<MatcherResult> mresvec;
  double thetaRes = 0.025; // was 0.0125*.5
 
  // clock_t t_ini, t_fin;
  // double secs;
  
  // t_ini = clock();
  _grid.hierarchicalSearch(mresvec, reducedScans, regions, thetaRes, maxScore, 0.5, 0.5, 0.2, 3);
  // t_fin = clock();
  // secs = (double)(t_fin - t_ini) / CLOCKS_PER_SEC;
  // printf("%.16g ms. Matcher results: %i\n", secs * 1000.0, (int) mresvec.size());

  if (mresvec.size()){
    Eigen::Vector3d adj=mresvec[0].transformation;
    SE2 transf;
    transf.setTranslation(Eigen::Vector2d(adj.x(), adj.y()));
    transf.setRotation(adj.z());
    //    cerr <<  " bestScore = " << mresvec[0].score << endl; 
    //cerr << "Found Loop Closure Edge. Transf: " << adj.x() << " " << adj.y() << " " << adj.z() << endl << endl;

    trel.push_back(transf);
  }

  if (trel.size())
    return true;

  return false;
}

bool ScanMatcher::globalMatching(OptimizableGraph::VertexSet& referenceVset, OptimizableGraph::Vertex* _referenceVertex, OptimizableGraph::Vertex* _currentVertex,  SE2 *trel, double maxScore){
 
  OptimizableGraph::VertexSet vset;
  vset.insert(_currentVertex);
  return globalMatching(referenceVset, _referenceVertex,  vset, _currentVertex, trel, maxScore);

}

bool ScanMatcher::globalMatching(OptimizableGraph::VertexSet& referenceVset, OptimizableGraph::Vertex* _referenceVertex, OptimizableGraph::VertexSet& currvset, OptimizableGraph::Vertex* _currentVertex, SE2 *trel, double maxScore){ 

  resetGrid();
  
  RawLaser::Point2DVector scansInRefVertex;
  transformPointsFromVSet(referenceVset, _referenceVertex, scansInRefVertex);
  _grid.addAndConvolvePoints<RawLaser::Point2DVector>(scansInRefVertex.begin(), scansInRefVertex.end(), _kernel);

  RawLaser::Point2DVector scansInCurVertex;
  transformPointsFromVSet(currvset, _currentVertex, scansInCurVertex);

  std::vector<MatcherResult> mresvec;
  //  clock_t t_ini, t_fin;
  //double secs;

  //t_ini = clock();
  //cerr << "Hierarchical Search: " << endl;

  double thetaRes = 0.025;
  Eigen::Vector3f lower(- 10, - 5, -M_PI);
  Eigen::Vector3f upper(+ 10, + 5,  M_PI); 

  Vector2dVector reducedScans;
  CharGrid::subsample(reducedScans, scansInCurVertex, 0.1);
  //cerr << "subsampling: " << scansInCurVertex.size() << " -> " << reducedScans.size() << endl;
  _grid.hierarchicalSearch(mresvec, reducedScans, lower, upper, thetaRes, maxScore, 0.5, 0.5, 0.2, 4);
  
  //t_fin = clock();

  //secs = (double)(t_fin - t_ini) / CLOCKS_PER_SEC;
  //printf("%.16g ms\n", secs * 1000.0);
  //cerr << "matcher results: " << mresvec.size();
  if (mresvec.size()){
    Eigen::Vector3d adj=mresvec[0].transformation;
    trel->setTranslation(Eigen::Vector2d(adj.x(), adj.y()));
    trel->setRotation(adj.z());
    //cerr <<  " bestScore = " << mresvec[0].score << endl; 

    // CharMatcher auxGrid = _LCGrid;


    // Vector2dVector transformedScan;
    // transformedScan.resize(reducedScans.size());
    // for (unsigned int i = 0; i<reducedScans.size(); i++){
    //   SE2 point;
    //   point.setTranslation(reducedScans[i]);
      
    //   SE2 transformedpoint = *trel * point;
    //   transformedScan[i] = transformedpoint.translation();
    // }
    // auxGrid = _LCGrid;
    // auxGrid.addPoints<RawLaser::Point2DVector>(transformedScan.begin(), transformedScan.end());

    // ofstream image;
    // std::stringstream filename;
    // filename << "matchedmapglobal" << currentVertex->id() << "_" << mresvec[0].score << ".ppm";
    // image.open(filename.str().c_str());
    // auxGrid.grid().saveAsPPM(image, false);
    return true;
  } 
  //cerr << endl;
  return false;
}

bool ScanMatcher::verifyMatching(OptimizableGraph::VertexSet& vset1, OptimizableGraph::Vertex* _referenceVertex1, 
				 OptimizableGraph::VertexSet& vset2, OptimizableGraph::Vertex* _referenceVertex2, 
				 SE2 trel12, double *score){ 

  VertexSE2* referenceVertex2=dynamic_cast<VertexSE2*>(_referenceVertex2);

  resetGrid();
  CharGrid auxGrid = _grid;

  //Transform points from vset2 in the reference of referenceVertex1 using trel12
  RawLaser::Point2DVector scansvset2inref1;
  for (OptimizableGraph::VertexSet::iterator it = vset2.begin(); it != vset2.end(); it++){
    VertexSE2 *vertex = (VertexSE2*) *it;
    RobotLaser* laserv = dynamic_cast<RobotLaser*>(vertex->userData());
    RawLaser::Point2DVector vscan = laserv->cartesian();
    SE2 trl = laserv->laserParams().laserPose;

    RawLaser::Point2DVector scanInRefVertex1;
    if (vertex->id() == referenceVertex2->id()){
      applyTransfToScan(trel12 * trl, vscan, scanInRefVertex1);
    }else{
      //Transform scans to the referenceVertex2 coordinates
      SE2 tref2_v = referenceVertex2->estimate().inverse() * vertex->estimate();
      applyTransfToScan(trel12 * tref2_v * trl, vscan, scanInRefVertex1);
    }
    scansvset2inref1.insert(scansvset2inref1.end(), scanInRefVertex1.begin(), scanInRefVertex1.end());
  }

  //Scans in vset1
  RawLaser::Point2DVector scansvset1;
  transformPointsFromVSet(vset1, _referenceVertex1, scansvset1);

  //Add local map from vset2 into the grid
  _grid.addAndConvolvePoints<RawLaser::Point2DVector>(scansvset2inref1.begin(), scansvset2inref1.end(), _kernel);
  //Find points from vset1 not explained by map vset2
  RawLaser::Point2DVector nonmatchedpoints;
  _grid.searchNonMatchedPoints(scansvset1, nonmatchedpoints, .3);

  //Add those points to a grid to count them
  auxGrid.addAndConvolvePoints<RawLaser::Point2DVector>(nonmatchedpoints.begin(), nonmatchedpoints.end(), _kernel);

  // ofstream image1;
  // std::stringstream filename1;
  // filename1 << "map2.ppm";
  // image1.open(filename1.str().c_str());
  // _LCGrid.grid().saveAsPPM(image1, false);

  // ofstream image2;
  // std::stringstream filename2;
  // filename2 << "mapnonmatched.ppm";
  // image2.open(filename2.str().c_str());
  // auxGrid.grid().saveAsPPM(image2, false);

  // //Just for saving the image
  // resetLCGrid();
  // _LCGrid.addAndConvolvePoints<RawLaser::Point2DVector>(scansvset1.begin(), scansvset1.end(), _LCKernel);
  // ofstream image3;
  // std::stringstream filename3;
  // filename3 << "map1.ppm";
  // image3.open(filename3.str().c_str());
  // _LCGrid.grid().saveAsPPM(image3, false);


  //Counting points around trel12
  Eigen::Vector2f lower(-.3+trel12.translation().x(), -.3+trel12.translation().y());
  Eigen::Vector2f upper(+.3+trel12.translation().x(), +.3+trel12.translation().y()); 
  
  auxGrid.countPoints(lower, upper, score);
  cerr << "Score: " << *score << endl;
  double threshold = 40.0;
  if (*score <= threshold)
    return true;
  
  return false;

}
