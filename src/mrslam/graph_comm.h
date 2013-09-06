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

#ifndef _GRAPH_COMM_H_
#define _GRAPH_COMM_H_

#include <sys/socket.h> /* socket specific definitions */
#include <arpa/inet.h> /* IP address conversion stuff */

#include <string>
#include <queue>


#include "mr_graph_slam.h"
#include <boost/thread.hpp>

#include "ros_handler.h"

#define SIM_EXPERIMENT 0
#define REAL_EXPERIMENT 1
#define BAG_EXPERIMENT 2

#define SIM_COMM_RANGE 5
#define COMM_TIME 10.0

typedef std::queue<StampedRobotMessage> msgQueue;

class GraphComm
{
 public:

  GraphComm(MRGraphSLAM* gslam, int idRobot, int nRobots, std::string base_addr, int typeExperiment);
  /*
  void init_network(SE2 gtPoses[]);
  void init_network(struct timeval pings[]);*/
  void init_network(RosHandler* rh);

 protected:
  bool inCommunicationRange(int r1, int r2);
  bool robotsInRange(std::vector<int>& robotsToSend);
  void send(RobotMessage* cmsg, int rto);
  void sendToThrd();
  RobotMessage* receive();
  void receiveFromThrd();
  void processQueueThrd();
  void init_threads();

  ////////////////////
  MRGraphSLAM* _gslam;

  int _iSock;
  int _idRobot;
  int _nRobots;
  int _typeExperiment;

  std::string _base_addr;

  ///////////////
  msgQueue _queue;
  boost::mutex _queueMutex;

  ///////////////
  RosHandler* _rh;

  //Threads
  boost::thread sthread;
  boost::thread rthread;
  boost::thread pthread;
};


#endif
