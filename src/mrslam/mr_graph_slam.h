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

#ifndef _MR_GRAPH_SLAM_H_
#define _MR_GRAPH_SLAM_H_

#include "slam/graph_slam.h"
#include "msg_factory.h"
#include "condensedGraphs/condensed_graph_buffer.h"
#include "mr_closure_buffer.h"

struct StampedRobotMessage{
  OptimizableGraph::Vertex* refVertex; //Current vertex when the msg was received
  RobotMessage* msg;
};

class MRGraphSLAM : public GraphSLAM
{
 public:

  MRGraphSLAM();
  
  void setIdRobot(int idRobot);
  void setInterRobotClosureParams(double maxScoreMR_, int minInliersMR_, int windowMRLoopClosure_);

  inline void setDetectRobotInRange(bool detectRobotInRange_){detectRobotInRange = detectRobotInRange_;}

  void addInterRobotData(StampedRobotMessage vmsg);
  
  void findInterRobotConstraints();

  ComboMessage*          constructComboMessage();
  CondensedGraphMessage* constructCondensedGraphMessage(int idRobotTo);
  GraphMessage*          constructGraphMessage(int idRobotTo);

  RobotMessage* createMsgfromCharArray(const char* buffer, size_t size);

 protected:

  CondensedGraphBuffer condensedGraphs;

  MRClosureBuffer interRobotClosures; //Contains already matched vertices + edges
  MRClosureBuffer interRobotVertices; //Contains non-matched vertices

  double maxScoreMR;
  int minInliersMR;
  int windowMRLoopClosure;

  bool detectRobotInRange;

  MessageFactory *factory;

  void addInterRobotData(GraphMessage* gmsg);
  void addInterRobotData(CondensedGraphMessage* cmsg);
  void addInterRobotData(ComboMessage* cmsg,   OptimizableGraph::Vertex* refVertex);

  void checkInterRobotClosures();
  void updateInterRobotClosures();

  VertexArrayMessage*    constructVertexArrayMessage(OptimizableGraph::VertexIDMap& vertices);
  EdgeArrayMessage*      constructEdgeArrayMessage(OptimizableGraph::EdgeSet& edges);
};

#endif
