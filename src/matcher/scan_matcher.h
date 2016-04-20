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

#ifndef _SCAN_MATCHER_H_
#define _SCAN_MATCHER_H_

#include "g2o/core/optimizable_graph.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/data/robot_laser.h"

#include "chargrid.h"

using namespace g2o;

class ScanMatcher{
 public:
  ScanMatcher();

  void initializeKernel(double resolution, double kernelRange);
  void initializeGrid(Eigen::Vector2f lowerLeft, Eigen::Vector2f upperRight, double resolution);
  void resetGrid();
 
  bool closeScanMatching(OptimizableGraph::VertexSet& vset, OptimizableGraph::Vertex* _originVertex, 
			 OptimizableGraph::Vertex* _currentVertex,  
			 SE2 *trel, double maxScore);

  bool scanMatchingLC(OptimizableGraph::VertexSet& referenceVset,  OptimizableGraph::Vertex* _referenceVertex, 
		      OptimizableGraph::Vertex* _currentVertex,  
		      std::vector<SE2>& trel, double maxScore);

  bool scanMatchingLC(OptimizableGraph::VertexSet& referenceVset,  OptimizableGraph::Vertex* _referenceVertex, 
		      OptimizableGraph::VertexSet& currvset, OptimizableGraph::Vertex* _currentVertex,  
		      std::vector<SE2>& trel, double maxScore);

  bool scanMatchingLChierarchical(OptimizableGraph::VertexSet& referenceVset,  OptimizableGraph::Vertex* _referenceVertex, 
				  OptimizableGraph::VertexSet& currvset, OptimizableGraph::Vertex* _currentVertex,  
				  std::vector<SE2>& trel, double maxScore);
  
  bool globalMatching(OptimizableGraph::VertexSet& referenceVset, OptimizableGraph::Vertex* _referenceVertex, 
		      OptimizableGraph::Vertex* _currentVertex,  
		      SE2 *trel, double maxScore);

  bool globalMatching(OptimizableGraph::VertexSet& referenceVset, OptimizableGraph::Vertex* _referenceVertex, 
		      OptimizableGraph::VertexSet& currvset, OptimizableGraph::Vertex* _currentVertex, 
		      SE2 *trel, double maxScore);


  bool verifyMatching(OptimizableGraph::VertexSet& vset1, OptimizableGraph::Vertex* _referenceVertex1, 
				   OptimizableGraph::VertexSet& vset2, OptimizableGraph::Vertex* _referenceVertex2, 
		      SE2 trel12, double *score);
  inline CharGrid grid() const {return _grid;}
  static void applyTransfToScan(SE2 transf, RawLaser::Point2DVector scan, RawLaser::Point2DVector& outScan);

 protected:
  CharGrid _grid;
  MatrixXChar _kernel;
  double _kernelRange;
  int _kscale;
};

#endif
