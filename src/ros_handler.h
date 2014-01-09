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

#ifndef _ROS_HANDLER_H_
#define _ROS_HANDLER_H_

#include "ros/ros.h"
#include "tf/tf.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

#include "cg_mrslam/Ping.h"
#include "cg_mrslam/SLAM.h"

#include "g2o/types/slam2d/se2.h"
#include "g2o/types/data/robot_laser.h"

#include "mrslam/msg_factory.h"

#define SIM_EXPERIMENT 0
#define REAL_EXPERIMENT 1
#define BAG_EXPERIMENT 2

using namespace g2o;

class RosHandler
{
 public:
  RosHandler(int idRobot, int nRobots, int typeExperiment);
  
  inline void useOdom(bool useOdom){_useOdom = useOdom;}
  inline void useLaser(bool useLaser){_useLaser = useLaser;}

  SE2 getOdom();
  RobotLaser* getLaser();
  
  inline SE2 getGroundTruth(int robot){return _gtPoses[robot];}
  inline ros::Time getTimeLastPing(int robot){return _timeLastPing[robot];}

  inline void setOdomTopic(std::string odomTopic) {_odomTopic = odomTopic;}
  inline void setScanTopic(std::string scanTopic) {_scanTopic = scanTopic;}

  void init();
  void run();

  void publishSentMsg(RobotMessage* msg);
  void publishReceivedMsg(RobotMessage* msg);
  void publishPing(int idRobotFrom);

 protected:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
  void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg, SE2 *gtpose);
  void pingCallback(const cg_mrslam::Ping::ConstPtr& msg);

  void createDSlamMsg(RobotMessage* msg, cg_mrslam::SLAM& dslamMsg);
  void createCondensedGraphMsg(CondensedGraphMessage* gmsg,  cg_mrslam::SLAM& dslamMsg);
  void createComboMsg(ComboMessage* cmsg, cg_mrslam::SLAM& dslamMsg);

  ////////////////////
  ros::NodeHandle _nh;

  //Subscribers
  ros::Subscriber _subOdom;
  ros::Subscriber _subScan;
  ros::Subscriber *_subgt;
  ros::Subscriber _subPing;

  //Publishers
  ros::Publisher _pubRecv;
  ros::Publisher _pubSent;
  ros::Publisher _pubPing;

  //Topics names
  std::string _odomTopic;
  std::string _scanTopic;
  
  int _idRobot;
  int _nRobots;
  int _typeExperiment;
  bool _useOdom, _useLaser;

  //ROS msgs
  nav_msgs::Odometry _odom;
  sensor_msgs::LaserScan _laserscan;
  SE2 *_gtPoses;

  ros::Time *_timeLastPing;

};

#endif
