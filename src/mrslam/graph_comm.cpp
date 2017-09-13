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

#include "graph_comm.h"

GraphComm::GraphComm (MRGraphSLAM* gslam, int idRobot, int nRobots, std::string base_addr, TypeExperiment typeExperiment){
  _typeExperiment = typeExperiment;

  _idRobot = idRobot;
  _nRobots = nRobots;

  _base_addr = base_addr;

  _gslam = gslam;

  std::stringstream my_addr;
  my_addr << base_addr << idRobot+1;
  std::cerr << "My address: " << my_addr.str() << std::endl;

  _iSock = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);

  struct sockaddr_in sockAddr;
  sockAddr.sin_family=AF_INET;
  sockAddr.sin_addr.s_addr=inet_addr(my_addr.str().c_str());
  sockAddr.sin_port=htons(42001);
  bind(_iSock,(struct sockaddr*)&sockAddr,sizeof(sockAddr));

}

void GraphComm::init_threads(){
  sthread = boost::thread(&GraphComm::sendToThrd, this);
  rthread = boost::thread(&GraphComm::receiveFromThrd, this);
  pthread = boost::thread(&GraphComm::processQueueThrd, this);
}

void GraphComm::init_network(RosHandler* rh){
  _rh = rh;
  init_threads();
}

bool GraphComm::inCommunicationRange(int r1, int r2){
  return distanceSE2(_rh->getGroundTruth(r1), _rh->getGroundTruth(r2)) < SIM_COMM_RANGE;
}

bool GraphComm::robotsInRange(std::vector<int>& robotsToSend){
  robotsToSend.clear();

  if (_typeExperiment == REAL){
    //Send to all... the message will arrive if they are in range
    for (int r = 0; r < _nRobots; r++){
      if (r != _idRobot) //Except to me!
	robotsToSend.push_back(r);
    }
  }else if (_typeExperiment == SIM){
    //Send if inCommunicationRange
    for (int r = 0; r < _nRobots; r++){
      if (r != _idRobot){
	//Looking for ground truth pose
	if (inCommunicationRange(_idRobot, r))
	    robotsToSend.push_back(r);
      }
    }
  }else if (_typeExperiment == BAG){
    //Send if recent ping
    ros::Time curr_time = ros::Time::now();
    for (int r = 0; r < _nRobots; r++){
      if (r != _idRobot){
	if ((curr_time.toSec() -_rh->getTimeLastPing(r).toSec()) < COMM_TIME){ //Less than COMM_TIME seconds since last ping
	  robotsToSend.push_back(r);
	}
      }
    }
  }

  return robotsToSend.size();
}

void GraphComm::send(RobotMessage* cmsg, int rto){
  std::stringstream to_addr;
  to_addr << _base_addr << rto +1;

  struct sockaddr_in toSockAddr;
  toSockAddr.sin_family=AF_INET;
  toSockAddr.sin_addr.s_addr=inet_addr(to_addr.str().c_str());
  toSockAddr.sin_port=htons(42001);
  
  char bufferc [MAX_LENGTH_MSG];
  char* c = cmsg->toCharArray(bufferc, MAX_LENGTH_MSG); //  create buffer;
  size_t sizebufc = (c) ? (c-bufferc):0;
 
  if (sizebufc){
    std::cerr << "Send info to robot: " << rto << ". Address: " << to_addr.str() << ". Sent: " << sizebufc  << " bytes" << std::endl;
    sendto(_iSock, &bufferc, sizebufc, 0, (struct sockaddr*) &toSockAddr, sizeof(toSockAddr));
    if (_typeExperiment == REAL)
      _rh->publishSentMsg(cmsg);
  }
}



void GraphComm::sendToThrd(){
  int lastSentVertex = -1;
  std::vector<int> robotsToSend;
  while(1){
    if (robotsInRange(robotsToSend)){
      if (_gslam->lastVertex()->id() != lastSentVertex){
	lastSentVertex = _gslam->lastVertex()->id();
	
	ComboMessage* cmsg = _gslam->constructComboMessage();
	//Send to robots in range
	for (size_t i = 0; i < robotsToSend.size(); i++){
	  int rto = robotsToSend[i];
	  send(cmsg, rto);
	}
      }

      for (size_t i = 0; i < robotsToSend.size(); i++){
	int rto = robotsToSend[i];

	CondensedGraphMessage* gmsg = _gslam->constructCondensedGraphMessage(rto);
	//GraphMessage* gmsg = _gslam->constructGraphMessage(rto);

	if (gmsg)
	  send(gmsg, rto);
      }
    }
    usleep(150000);
  }
}

RobotMessage* GraphComm::receive(){
  struct sockaddr_in toSockAddr;
  int toSockAddrLen=sizeof(struct sockaddr_in);
  
  int sizebuf = MAX_LENGTH_MSG;
  char buffer[sizebuf];

  int nbytes = recvfrom(_iSock, &buffer, sizebuf ,0,(struct sockaddr*)&toSockAddr, (socklen_t*)&toSockAddrLen);
  fprintf(stderr, "Received %i bytes.\n", nbytes);

  //////////////////
  //Deserialize data
  RobotMessage *msg = _gslam->createMsgfromCharArray(buffer, nbytes);

  return msg;
}

void GraphComm::receiveFromThrd(){

  while(1){
    //////////////////
    //Receive data
    RobotMessage* msg = receive();

    fprintf(stderr, "Received info from: %i\n", msg->robotId());
    if (_typeExperiment == REAL){
      _rh->publishReceivedMsg(msg);
      _rh->publishPing(msg->robotId());
    }

    StampedRobotMessage vmsg;
    vmsg.msg = msg;
    vmsg.refVertex = _gslam->lastVertex();

    boost::mutex::scoped_lock lock(_queueMutex);
    _queue.push(vmsg);
  }
}

void GraphComm::processQueueThrd(){

  while(1){
    if (!_queue.empty()){
      boost::mutex::scoped_lock lock(_queueMutex);

      StampedRobotMessage vmsg = _queue.front();
      _gslam->addInterRobotData(vmsg);

      _queue.pop();
    }else
      usleep(10000);
  }
}





