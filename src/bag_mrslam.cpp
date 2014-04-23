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

#include "g2o/stuff/command_args.h"

#include <string>
#include <sstream> 

#include "mrslam/mr_graph_slam.h"
#include "mrslam/graph_comm.h"
#include "ros_handler.h"
#include "graph_ros_publisher.h"

using namespace g2o;

int main(int argc, char **argv)
{

  CommandArgs arg;
  double resolution;
  double maxScore, maxScoreMR;
  double kernelRadius;
  int  minInliers, minInliersMR;
  int windowLoopClosure, windowMRLoopClosure;
  double inlierThreshold;
  int idRobot;
  int nRobots;
  std::string outputFilename;
  std::string odometryTopic, scanTopic, fixedFrame;

  arg.param("resolution",  resolution, 0.025, "resolution of the matching grid");
  arg.param("maxScore",    maxScore, 0.15,     "score of the matcher, the higher the less matches");
  arg.param("kernelRadius", kernelRadius, 0.2,  "radius of the convolution kernel");
  arg.param("minInliers",    minInliers, 7,     "min inliers");
  arg.param("windowLoopClosure",  windowLoopClosure, 10,   "sliding window for loop closures");
  arg.param("inlierThreshold",  inlierThreshold, 2.,   "inlier threshold");
  arg.param("idRobot", idRobot, 0, "robot identifier" );
  arg.param("nRobots", nRobots, 1, "number of robots" );
  arg.param("maxScoreMR",    maxScoreMR, 0.15,  "score of the intra-robot matcher, the higher the less matches");
  arg.param("minInliersMR",    minInliersMR, 5,     "min inliers for the intra-robot loop closure");
  arg.param("windowMRLoopClosure",  windowMRLoopClosure, 15,   "sliding window for the intra-robot loop closures");
  arg.param("odometryTopic", odometryTopic, "odom", "odometry ROS topic");
  arg.param("scanTopic", scanTopic, "scan", "scan ROS topic");
  arg.param("fixedFrame", fixedFrame, "odom", "fixed frame to visualize the graph with ROS Rviz");
  arg.param("o", outputFilename, "", "file where to save output");
  arg.parseArgs(argc, argv);

  ros::init(argc, argv, "bag_mrslam");
  
  RosHandler rh(idRobot, nRobots, BAG_EXPERIMENT);
  rh.setOdomTopic(odometryTopic);
  rh.setScanTopic(scanTopic);
  rh.useOdom(true);
  rh.useLaser(true);

  rh.init();   //Wait for initial odometry and laserScan
  rh.run();

  //For estimation
  SE2 currEst = rh.getOdom();
  std::cout << "My initial position is: " << currEst.translation().x() << " " << currEst.translation().y() << " " << currEst.rotation().angle() << std::endl;
  SE2 odomPosk_1 = currEst;
  std::cout << "My initial odometry is: " << odomPosk_1.translation().x() << " " << odomPosk_1.translation().y() << " " << odomPosk_1.rotation().angle() << std::endl;

  //Graph building
  MRGraphSLAM gslam;
  gslam.setIdRobot(idRobot);
  int baseId = 10000;
  gslam.setBaseId(baseId);
  gslam.init(resolution, kernelRadius, windowLoopClosure, maxScore, inlierThreshold, minInliers);
  gslam.setInterRobotClosureParams(maxScoreMR, minInliersMR, windowMRLoopClosure);

  RobotLaser* rlaser = rh.getLaser();

  gslam.setInitialData(odomPosk_1, rlaser);

  GraphRosPublisher graphPublisher(gslam.graph(), fixedFrame);

  ////////////////////
  //Setting up network
  std::string base_addr = "127.0.0.";
  GraphComm gc(&gslam, idRobot, nRobots, base_addr, BAG_EXPERIMENT);
  gc.init_network(&rh);

  ros::Rate loop_rate(10);

  while (ros::ok()){
    ros::spinOnce();

    SE2 odomPosk = rh.getOdom(); //current odometry
    SE2 relodom = odomPosk_1.inverse() * odomPosk;
    currEst *= relodom;

    odomPosk_1 = odomPosk;

    if((distanceSE2(gslam.lastVertex()->estimate(), currEst) > 0.25) || 
       (fabs(gslam.lastVertex()->estimate().rotation().angle()-currEst.rotation().angle()) > M_PI_4)){
      //Add new data
      RobotLaser* laseri = rh.getLaser();

      gslam.addDataSM(odomPosk, laseri);
      gslam.findConstraints();
      gslam.findInterRobotConstraints();

      gslam.optimize(5);

      currEst = gslam.lastVertex()->estimate();
      char buf[100];
      sprintf(buf, "robot-%i-%s", idRobot, outputFilename.c_str());
      gslam.saveGraph(buf);

      //Publish graph to visualize it on Rviz
      graphPublisher.publishGraph();

    }
    
    loop_rate.sleep();
  }

  cerr << "Last Optimization...";
  gslam.optimize(5);

  char buf[100];
  sprintf(buf, "robot-%i-%s", idRobot, outputFilename.c_str());
  gslam.saveGraph(buf);
  cerr << "Done" << endl;
  
  return 0;
}
