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

#include "graph_ros_publisher.h"

GraphRosPublisher::GraphRosPublisher(OptimizableGraph* graph, string fixedFrame){
  _graph = graph;
  _fixedFrame = fixedFrame;

  _pubtj = _nh.advertise<geometry_msgs::PoseArray>("trajectory", 1);
  _publm = _nh.advertise<sensor_msgs::PointCloud>("lasermap", 1);
}

void GraphRosPublisher::publishGraph(){

  assert(_graph && "Cannot publish: undefined graph");

  geometry_msgs::PoseArray traj;
  sensor_msgs::PointCloud pcloud;
  traj.poses.resize(_graph->vertices().size());
  pcloud.points.clear();
  int i = 0;
  for (OptimizableGraph::VertexIDMap::iterator it=_graph->vertices().begin(); it!=_graph->vertices().end(); ++it) {
    VertexSE2* v = (VertexSE2*) (it->second);
    traj.poses[i].position.x = v->estimate().translation().x();
    traj.poses[i].position.y = v->estimate().translation().y();
    traj.poses[i].position.z = 0;
    traj.poses[i].orientation = tf::createQuaternionMsgFromYaw(v->estimate().rotation().angle());

    RobotLaser *laser = dynamic_cast<RobotLaser*>(v->userData());
    if (laser){
      RawLaser::Point2DVector vscan = laser->cartesian();
      SE2 trl = laser->laserParams().laserPose;
      SE2 transf = v->estimate() * trl;
      RawLaser::Point2DVector wscan;
      ScanMatcher::applyTransfToScan(transf, vscan, wscan);
	  
      size_t s= 0;
      while ( s<wscan.size()){
	geometry_msgs::Point32 point;
	point.x = wscan[s].x();
	point.y = wscan[s].y();
	pcloud.points.push_back(point);
	
	s = s+10;
      }
    }
    i++;
  }
  
  traj.header.frame_id = _fixedFrame;
  pcloud.header.frame_id = traj.header.frame_id;
  _publm.publish(pcloud);
  _pubtj.publish(traj);

}
