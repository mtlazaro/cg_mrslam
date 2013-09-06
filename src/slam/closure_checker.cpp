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

#include "closure_checker.h"

LoopClosureChecker::LoopClosureChecker(){}

void LoopClosureChecker::init(OptimizableGraph::VertexIDMap& movableRegion, OptimizableGraph::EdgeSet& closingEdges, double inlierThreshold) {
  _localVertices = movableRegion;
  _closuresToCheck = closingEdges;
  _inlierThreshold = inlierThreshold;
  _bestInliers = 0;
  _bestChi2 = std::numeric_limits<double>::max();
  _tempResult.clear();
  _bestResult.clear();
  for (OptimizableGraph::EdgeSet::iterator it=closingEdges.begin(); it!=closingEdges.end(); it++){
    OptimizableGraph::Edge* e=(OptimizableGraph::Edge*)(*it);
    _tempResult.insert(make_pair(e, std::numeric_limits<double>::max()));
    _bestResult.insert(make_pair(e, std::numeric_limits<double>::max()));
  }
}
  
void LoopClosureChecker::check(const std::string& matchingType){
  if (matchingType == "2dPose") {
    for (OptimizableGraph::EdgeSet::iterator it=_closuresToCheck.begin(); it!=_closuresToCheck.end(); it++){
      OptimizableGraph::Edge* e=(OptimizableGraph::Edge*)(*it);
      OptimizableGraph::EdgeSet eset;
      eset.insert(e);
      checkAndUpdateBest(eset, matchingType);
    }
  } else {
    assert (0 && "wrong matching strategy selected");
  }
}

// this is gonna be unelegant
void LoopClosureChecker::checkAndUpdateBest(OptimizableGraph::EdgeSet& eset, const std::string& matchingType) {
  applyZeroErrorTransform(eset, matchingType);
  // count the chi2 and the inliers;
  int inliers = 0;
  double totalChi2 = 0;
  for (EdgeDoubleMap::iterator it= _tempResult.begin(); it!= _tempResult.end(); it++){
    if (it->second < _inlierThreshold) {
      inliers ++;
      totalChi2 += it->second;
    }
  }
  if (inliers > _bestInliers || (inliers == _bestInliers && totalChi2 < _bestChi2) ){
    _bestInliers = inliers;
    _bestChi2 = totalChi2;
    _bestResult = _tempResult;
  } 
}


void LoopClosureChecker::applyZeroErrorTransform(OptimizableGraph::EdgeSet& eset, const std::string& matchingType){
  if (matchingType == "2dPose"){
    assert(eset.size()==1);
    EdgeSE2* e=dynamic_cast<EdgeSE2*>(*eset.begin());
    assert (e && "Wrong edge type for 2d pose matching");

    // determine which of the two vertices is the one that moves
    VertexSE2* root = 0;
    VertexSE2* vfrom=dynamic_cast<VertexSE2*>(e->vertices()[0]);
    VertexSE2* vto=dynamic_cast<VertexSE2*>(e->vertices()[1]);
    OptimizableGraph::VertexIDMap::iterator it = _localVertices.find(vfrom->id());
    if (it != _localVertices.end())
      root = vfrom;

    it = _localVertices.find(vto->id());
    if (it != _localVertices.end())
      root = vto;
	
    assert (root && "the loop closure does not have any vertex in the floating part of the map");
      
    // determine the transformation that would result in a zero error of the constraint
    SE2  newRootPose(0.,0.,0.);
    if (root == vfrom){
      newRootPose = vto->estimate()*e->measurement().inverse();
    } else {
      newRootPose = vfrom->estimate()*e->measurement();
    }
    SE2  motion = newRootPose * root->estimate().inverse();

    typedef std::map<VertexSE2*, SE2, std::less<VertexSE2*>, Eigen::aligned_allocator<std::pair<const VertexSE2*, SE2> > > VertexSE2Map;
    VertexSE2Map oldPoses;
    // now push all the vertices in the moving map and apply the transform
    for (OptimizableGraph::VertexIDMap::iterator it = _localVertices.begin(); it!=_localVertices.end(); it++){
      VertexSE2* v = (VertexSE2*)it->second;
      oldPoses[v]=v->estimate();
      v->push();
      v->setEstimate(motion*v->estimate());
    }
    for (EdgeDoubleMap::iterator it= _tempResult.begin(); it!= _tempResult.end(); it++){
      OptimizableGraph::Edge* e = (OptimizableGraph::Edge*) (it->first);
      e->computeError();
      VertexSE2* v = 0;
      VertexSE2* vfrom=dynamic_cast<VertexSE2*>(e->vertices()[0]);
      VertexSE2* vto=dynamic_cast<VertexSE2*>(e->vertices()[1]);
      if (_localVertices.count(vfrom->id()))
	v=vfrom;
      else
	v=vto;
      SE2 estBefore = oldPoses[v];
      SE2 delta = estBefore.inverse()*v->estimate();
      double thetaDifference = delta.rotation().angle();
      it->second = e->chi2() + 0 * thetaDifference * thetaDifference;
    }
    for (OptimizableGraph::VertexIDMap::iterator it = _localVertices.begin(); it!=_localVertices.end(); it++){
      VertexSE2* v=(VertexSE2*)(it->second);
      v->pop();
    }
  }
}
