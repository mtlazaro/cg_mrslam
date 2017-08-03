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

#ifndef _GRAPH_ROS_PUBLISHER_H_
#define _GRAPH_ROS_PUBLISHER_H_

#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Point32.h"

#include "g2o/core/optimizable_graph.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/data/robot_laser.h"

#include "slam/graph_slam.h"

#include <string>

using namespace g2o;
using namespace std;

class GraphRosPublisher
{
 public:
  GraphRosPublisher(OptimizableGraph* graph, string mapFrame, string odomFrame, SE2 pose = SE2());

  void publishGraph();
  void publishMapTransform(SE2 lastVertexEstimate, SE2 lastOdom);


 protected:
  ros::NodeHandle _nh;

  ros::Publisher _pubtj;
  ros::Publisher _publm;
  tf::TransformBroadcaster _broadcaster;


  SE2 _initialGroundTruth;
  OptimizableGraph* _graph;
  string _mapFrame, _odomFrame;
};

#endif
