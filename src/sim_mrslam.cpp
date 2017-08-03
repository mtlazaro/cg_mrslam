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
#include "ros_utils/ros_handler.h"
#include "ros_utils/graph_ros_publisher.h"

#include "map_creation/graph2occupancy.h"
#include "map_creation/occupancy_map_server.h"

using namespace g2o;

//Log files
int logData;
ofstream timesfile, bytesfile;

#include <sys/time.h>
double timeval_diff(struct timeval *a, struct timeval *b)
{
  return
    (double)(a->tv_sec + (double)a->tv_usec/1000000) -
    (double)(b->tv_sec + (double)b->tv_usec/1000000);
}

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
  std::string odometryTopic, scanTopic, odomFrame, mapFrame, baseFrame;
  bool publishTransform;

  float localizationAngularUpdate, localizationLinearUpdate;
  float maxRange, usableRange;

  arg.param("resolution",  resolution, 0.025, "resolution of the matching grid");
  arg.param("maxScore",    maxScore, 0.15,     "score of the matcher, the higher the less matches");
  arg.param("kernelRadius", kernelRadius, 0.2,  "radius of the convolution kernel");
  arg.param("minInliers",    minInliers, 7,     "min inliers");
  arg.param("windowLoopClosure",  windowLoopClosure, 10,   "sliding window for loop closures");
  arg.param("inlierThreshold",  inlierThreshold, 2.,   "inlier threshold");
  arg.param("idRobot", idRobot, 0, "robot identifier" );
  arg.param("nRobots", nRobots, 1, "number of robots" );
  arg.param("angularUpdate", localizationAngularUpdate, M_PI_4, "angular rotation interval for updating the graph, in radians");
  arg.param("linearUpdate", localizationLinearUpdate, 0.25, "linear translation interval for updating the graph, in meters");
  arg.param("maxScoreMR",    maxScoreMR, 0.15,  "score of the intra-robot matcher, the higher the less matches");
  arg.param("minInliersMR",    minInliersMR, 5,     "min inliers for the intra-robot loop closure");
  arg.param("windowMRLoopClosure",  windowMRLoopClosure, 10,   "sliding window for the intra-robot loop closures");
  arg.param("logData",  logData, 0,   "to log computation times, transmition overload and ground truth map");
  arg.param("odometryTopic", odometryTopic, "odom", "odometry ROS topic");
  arg.param("scanTopic", scanTopic, "scan", "scan ROS topic");
  arg.param("odomFrame", odomFrame, "odom", "odom frame");
  arg.param("mapFrame", mapFrame, "map", "map frame");
  arg.param("baseFrame", baseFrame, "/base_link", "base robot frame");
  arg.param("publishTransform", publishTransform, false, "Publish map transform");
  arg.param("o", outputFilename, "", "file where to save output");
  arg.parseArgs(argc, argv);

  //map parameters
  float mapResolution = 0.05;
  float occupiedThrehsold = 0.65; 
  float rows = 0;
  float cols = 0;
  float gain = 3.0;
  float squareSize = 0;
  float angle = 0.0;
  float freeThrehsold = 0.196;


  ros::init(argc, argv, "sim_mrslam");

  RosHandler rh(idRobot, nRobots, SIM_EXPERIMENT);
  rh.setBaseFrame(baseFrame);
  rh.useOdom(true);
  rh.useLaser(true);
  rh.init();   //Wait for initial ground-truth position, odometry and laserScan
  rh.run();

  maxRange = rh.getLaserMaxRange();
  usableRange = maxRange;
  
  for (int r = 0; r<nRobots; r++){
    std::cerr << "Ground Truth robot " << r << ": " << rh.getGroundTruth(r).translation().x() << " " << rh.getGroundTruth(r).translation().y() << " " << rh.getGroundTruth(r).rotation().angle() << std::endl;
  }

  //For estimation
  SE2 currEst = rh.getGroundTruth(idRobot);
  std::cerr << "My initial position is: " << currEst.translation().x() << " " << currEst.translation().y() << " " << currEst.rotation().angle() << std::endl;
  SE2 odomPosk_1 = rh.getOdom();
  std::cerr << "My initial odometry is: " << odomPosk_1.translation().x() << " " << odomPosk_1.translation().y() << " " << odomPosk_1.rotation().angle() << std::endl;

  if (logData){
    //Log files
    std::stringstream timesfilename, bytesfilename;
    timesfilename << "times-robot-" << idRobot << "-" << outputFilename;
    bytesfilename << "bytes-robot-" << idRobot << "-" << outputFilename;
    
    timesfile.open(timesfilename.str().c_str());
    bytesfile.open(bytesfilename.str().c_str());
  }


  //Graph building
  MRGraphSLAM gslam, gtgraph;
  gslam.setIdRobot(idRobot);
  int baseId = 10000;
  gslam.setBaseId(baseId);
  gslam.init(resolution, kernelRadius, windowLoopClosure, maxScore, inlierThreshold, minInliers);
  gslam.setInterRobotClosureParams(maxScoreMR, minInliersMR, windowMRLoopClosure);

  //Map building
  cv::Mat occupancyMap;
  Eigen::Vector2f mapCenter;
  
  Graph2occupancy mapCreator(gslam.graph(), &occupancyMap, currEst, mapResolution, occupiedThrehsold, rows, cols, maxRange, usableRange, gain, squareSize, angle, freeThrehsold);
  OccupancyMapServer mapServer(&occupancyMap, idRobot, SIM_EXPERIMENT, mapFrame, occupiedThrehsold, freeThrehsold);


  RobotLaser* rlaser = rh.getLaser();

  gslam.setInitialData(currEst, odomPosk_1, rlaser);
  if (logData){
    gtgraph.setIdRobot(idRobot);
    gtgraph.setBaseId(baseId);
    gtgraph.setInitialData(currEst, rlaser);
  }
  
  GraphRosPublisher graphPublisher(gslam.graph(), mapFrame, odomFrame, currEst);
  if (publishTransform)
    graphPublisher.publishMapTransform(gslam.lastVertex()->estimate(), odomPosk_1);

  char buf[100];
  sprintf(buf, "robot-%i-%s", idRobot, outputFilename.c_str());
  ofstream ofmap(buf);
  gslam.graph()->saveVertex(ofmap, gslam.lastVertex());

  ////////////////////
  //Setting up network
  std::string base_addr = "127.0.0.";
  GraphComm gc(&gslam, idRobot, nRobots, base_addr, SIM_EXPERIMENT);
  gc.init_network(&rh);


  mapCreator.computeMap();
  
  mapCenter = mapCreator.getMapCenter();
  mapServer.setOffset(mapCenter);
  mapServer.setResolution(mapResolution);
  mapServer.publishMapMetaData();
  mapServer.publishMap();


  ros::Rate loop_rate(10);
  while (ros::ok()){
    ros::spinOnce();

    SE2 odomPosk = rh.getOdom(); //current odometry
    SE2 relodom = odomPosk_1.inverse() * odomPosk;
    currEst *= relodom;

    odomPosk_1 = odomPosk;

    if((distanceSE2(gslam.lastVertex()->estimate(), currEst) > localizationLinearUpdate) || 
       (fabs(gslam.lastVertex()->estimate().rotation().angle()-currEst.rotation().angle()) > localizationAngularUpdate)){
      //Add new data
      RobotLaser* laseri = rh.getLaser();

      gslam.addDataSM(odomPosk, laseri);
      gslam.findConstraints();
      gslam.findInterRobotConstraints();
      
      struct timeval t_ini, t_fin;
      double secs;
      gettimeofday(&t_ini, NULL);
      gslam.optimize(5);
      gettimeofday(&t_fin, NULL);

      secs = timeval_diff(&t_fin, &t_ini);
      printf("Optimization took %.16g milliseconds\n", secs * 1000.0);

      currEst = gslam.lastVertex()->estimate();
      char buf[100];
      sprintf(buf, "robot-%i-%s", idRobot, outputFilename.c_str());
      gslam.saveGraph(buf);
 
      if (logData){
      	gtgraph.addData(rh.getGroundTruth(idRobot), laseri);
      	timesfile << gslam.graph()->edges().size() << "\t" << ros::Time::now() << "\t" << secs*1000.0 << endl;
      	char buf2[100];
      	sprintf(buf2, "gt-robot-%i-%s", idRobot, outputFilename.c_str());
      	gtgraph.saveGraph(buf2);
      }

      //Publish graph to visualize it on Rviz
      graphPublisher.publishGraph();
      //Publish map transform with corrected estimate
      if (publishTransform)
	     graphPublisher.publishMapTransform(gslam.lastVertex()->estimate(), odomPosk_1);

        mapCreator.computeMap();
        mapCenter = mapCreator.getMapCenter();
        mapServer.setOffset(mapCenter);

    }

    else {
      //Publish map transform with last corrected estimate + odometry drift
      if (publishTransform)
	     graphPublisher.publishMapTransform(currEst, odomPosk_1);
    }

    mapServer.publishMapMetaData();
    mapServer.publishMap();
    
    loop_rate.sleep();
  }
  
  cerr << "Last Optimization...";
  gslam.optimize(5);
  gslam.saveGraph(buf);
  cerr << "Done" << endl;

  return 0;
}
