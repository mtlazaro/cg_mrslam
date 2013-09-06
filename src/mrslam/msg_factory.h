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

#ifndef _MSG_FACTORY_H_
#define _MSG_FACTORY_H_

#include <iostream>
#include <vector>
#include <map>
#include <typeinfo>
#include <cstdlib>
#include <cstring>

#include <cassert>


using namespace std;


template <typename T> 
inline char* _toCharArray(const T& d, char* buf, int bsize){
  const char* c=reinterpret_cast<const char*>(&d);
  char* begin=buf;
  char* end=buf+sizeof(T);
  if ((end-begin) > bsize ) {
    //cerr << "Not enough space in buf!" << endl;
    return 0;
  }
  while(begin<end){
    *begin=*c;
    begin++;
    c++;
  }
  return end;
}

template <typename T> 
inline const char* _fromCharArray(T& d, const char* buf, int bsize = 0 ){
  char* c=reinterpret_cast<char*>(&d);
  const char* begin=buf;
  const char* end=buf+sizeof(T);
  if (bsize>0 && (end-begin) > bsize ) {
    return 0;
  }
  while(begin<end){
    *c = *begin;
    begin++;
    c++;
  }
  return end;
}

template <> 
inline const char* _fromCharArray<double>(double& d, const char* buf, int bsize){
  float f;
  char* c=reinterpret_cast<char*>(&f);
  const char* begin=buf;
  const char* end=buf+sizeof(float);
  if (bsize>0 && (end-begin) > bsize ) {
    return 0;
  }
  while(begin<end){
    *c = *begin;
    begin++;
    c++;
  }
  d=f;
  return end;
}

template <> 
inline char* _toCharArray<double>(const double& d, char* buf, int bsize){
  float f = d;
  const char* c=reinterpret_cast<const char*>(&f);
  char* begin=buf;
  char* end=buf+sizeof(float);
  if ((end-begin) > bsize ) {
    //cerr << "Not enough space in buf!" << endl;
    return 0;
  }
  while(begin<end){
    *begin=*c;
    begin++;
    c++;
  }
  return end;
}


#define MAX_LENGTH_MSG 100000

struct MessageFactory;

struct RobotMessage{
  RobotMessage(int robotId_ = -1){
    _robotId = robotId_;
  }

  // serializes the message in the buffer
  virtual char* toCharArray(char* buffer, size_t bsize, bool skipHeader = false) const;

  // fills in the data fields in the buffer
  virtual const char* fromCharArray(const char* buffer, size_t bsize= 0, bool skipHeader = false);

  virtual ~RobotMessage(){}
  void setRobotId(int rid) { _robotId = rid;}
  int robotId() const { return _robotId;}
  static int _type() { return 0;}
  virtual int type() const = 0;

protected:
  int _robotId;
};


struct VertexArrayMessage: public virtual RobotMessage{
  struct VSE2Data{
    int id;
    double estimate[3];
  };

 VertexArrayMessage(int robotId_ = -1) : RobotMessage(robotId_){}

  // serializes the message in the buffer
  virtual char* toCharArray(char* buffer, size_t bsize, bool skipHeader=false) const;

  // fills in the data fields in the buffer
  virtual const char* fromCharArray(const char* buffer, size_t bsize= 0, bool skipHeader=false);
  
  std::vector<VSE2Data> vertexVector;
  //protected:
  static int _type() { return 1;}
  virtual int type() const {return _type();}

};

struct RobotLaserMessage: public virtual RobotMessage{
 RobotLaserMessage(int robotId_ = -1) : RobotMessage(robotId_){}

  // serializes the message in the buffer
  virtual char* toCharArray(char* buffer, size_t bsize, bool skipHeader=false) const;

  // fills in the data fields in the buffer
  virtual const char* fromCharArray(const char* buffer, size_t bsize= 0, bool skipHeader=false);

  int nodeId;
  std::vector<double> readings;
  //laser params
  double minangle;
  double angleincrement;
  double maxrange;
  double accuracy;
  
  //protected:
  static int _type() { return 2;}
  virtual int type() const {return _type();}
};

struct ComboMessage : public VertexArrayMessage, RobotLaserMessage{
 ComboMessage(int robotId_ = -1) :
  VertexArrayMessage(robotId_),
    RobotLaserMessage(robotId_)
      {
	
      }
  virtual char* toCharArray(char* buffer, size_t bsize, bool skipHeader = false) const;

  virtual const char* fromCharArray(const char* buffer, size_t bsize= 0, bool skipHeader = false);

  //protected:
  static int _type() { return 4;}
  virtual int type() const {return _type();}
};

struct EdgeArrayMessage: public virtual RobotMessage{
  struct ESE2Data{
    int idfrom;
    int idto;
    double estimate[3];
    double information[6];
  };
  
 EdgeArrayMessage(int robotId_ = -1) : RobotMessage(robotId_){}
  
  // serializes the message in the buffer
  virtual char* toCharArray(char* buffer, size_t bsize, bool skipHeader=false) const;
  
  // fills in the data fields in the buffer
  virtual const char* fromCharArray(const char* buffer, size_t bsize= 0, bool skipHeader=false);
  
  std::vector<ESE2Data> edgeVector;
  //protected:
  static int _type() { return 5;}
  virtual int type() const {return _type();}

};

struct ClosuresMessage: public virtual RobotMessage{
  
 ClosuresMessage(int robotId_ = -1) : RobotMessage(robotId_){}
  
  // serializes the message in the buffer
  virtual char* toCharArray(char* buffer, size_t bsize, bool skipHeader=false) const;
  
  // fills in the data fields in the buffer
  virtual const char* fromCharArray(const char* buffer, size_t bsize= 0, bool skipHeader=false);
  
  std::vector<int> closures;
  //protected:
  static int _type() { return 6;}
  virtual int type() const {return _type();}

};

struct CondensedGraphMessage : public EdgeArrayMessage, ClosuresMessage{
 CondensedGraphMessage(int robotId_ = -1) :
  EdgeArrayMessage(robotId_),
    ClosuresMessage(robotId_)
    {

    }
  virtual char* toCharArray(char* buffer, size_t bsize, bool skipHeader = false) const;

  virtual const char* fromCharArray(const char* buffer, size_t bsize= 0, bool skipHeader = false);

  //protected:
  static int _type() { return 7;}
  virtual int type() const {return _type();}
};

struct GraphMessage : public VertexArrayMessage, EdgeArrayMessage, ClosuresMessage{
 GraphMessage(int robotId_ = -1) :
  VertexArrayMessage(robotId_),
    EdgeArrayMessage(robotId_),
    ClosuresMessage(robotId_)
    {
      
    }
  virtual char* toCharArray(char* buffer, size_t bsize, bool skipHeader = false) const;
  
  virtual const char* fromCharArray(const char* buffer, size_t bsize= 0, bool skipHeader = false);
  
  //protected:
  static int _type() { return 8;}
  virtual int type() const {return _type();}
};

struct MessageFactory{

protected:
  struct BaseMessageCreator {
    BaseMessageCreator(){};
    virtual int msgType() const = 0; 
    virtual RobotMessage* constructMessage() = 0;
    int _msgType;
  };

  template <typename T>
  struct MessageCreator : public BaseMessageCreator {
  MessageCreator() : BaseMessageCreator() {}
    virtual int msgType() const {
      return T::_type();
    }
    virtual RobotMessage* constructMessage() {
      return new T();
    }
  };


  std::map<int,BaseMessageCreator*> _msgCreatorMap;

public:
  size_t size() const {return _msgCreatorMap.size();}
  template <typename T>
  void registerMessageType(){
    int t=T::_type();
    std::map<int,BaseMessageCreator*>::iterator it = _msgCreatorMap.find(t);
    if (it!=_msgCreatorMap.end()) {
      assert ( 0 && "Message alteady registered in factory" );
    }
    MessageCreator<T> * creator = new MessageCreator<T>();
    _msgCreatorMap.insert(make_pair(t, creator));
  }

  RobotMessage* constructMessage(int t);
  RobotMessage* fromCharArray(const char* buf, size_t size);
  
};

#endif
