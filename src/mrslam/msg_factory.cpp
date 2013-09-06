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

#include "msg_factory.h"

char* RobotMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer;
  if (skipHeader)
    return c;
  c = _toCharArray(type(), c, bsize);
  c = _toCharArray(_robotId, c, bsize);
  return c;
}

// fills in the data fields in the buffer
const char* RobotMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  if (skipHeader)
    return c;
  int t;
  c = _fromCharArray(t, c);
  c = _fromCharArray(_robotId, c);
  assert (t==type() && "FATAL, TYPE MISMATCH");
  return c;
}

// serializes the message in the buffer
char* VertexArrayMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer;
  size_t free = bsize;

  c = RobotMessage::toCharArray(c, free, skipHeader);
  free = (c) ? bsize - (c - buffer) : 0;

  size_t s =vertexVector.size();
  c = _toCharArray<size_t>(s,c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  for (size_t i =0; i< vertexVector.size(); i++){
    c = _toCharArray(vertexVector[i].id, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(vertexVector[i].estimate[0], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(vertexVector[i].estimate[1], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(vertexVector[i].estimate[2], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
  }

  return c;
}

// fills in the data fields in the buffer
const char* VertexArrayMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  c = RobotMessage::fromCharArray(c, bsize, skipHeader);
  size_t s;
  c = _fromCharArray(s,c);
  vertexVector.resize(s);
  for (size_t i =0; i< vertexVector.size(); i++){
    c = _fromCharArray(vertexVector[i].id, c);
    c = _fromCharArray(vertexVector[i].estimate[0], c);
    c = _fromCharArray(vertexVector[i].estimate[1], c);
    c = _fromCharArray(vertexVector[i].estimate[2], c);
  }
  return c;
}

// serializes the message in the buffer
char* RobotLaserMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer; 
  size_t free = bsize;

  c = RobotMessage::toCharArray(c, free, skipHeader);
  free = (c) ? bsize - (c - buffer) : 0;

  c = _toCharArray(nodeId,c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  size_t s = readings.size();
  c = _toCharArray(s,c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  for (size_t i =0; i< readings.size(); i++){
    c = _toCharArray(readings[i], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
  }

  c = _toCharArray(minangle,c, free);
  free = (c) ? bsize - (c - buffer) : 0;
  c = _toCharArray(angleincrement,c, free);
  free = (c) ? bsize - (c - buffer) : 0;
  c = _toCharArray(maxrange,c, free);
  free = (c) ? bsize - (c - buffer) : 0;
  c = _toCharArray(accuracy,c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  return c;
}

// fills in the data fields in the buffer
const char* RobotLaserMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  c = RobotMessage::fromCharArray(c, bsize, skipHeader);
  c = _fromCharArray(nodeId,c);
  size_t s;
  c = _fromCharArray(s,c);
  readings.resize(s);
  for (size_t i =0; i< readings.size(); i++){
    c = _fromCharArray(readings[i], c);
  }

  c = _fromCharArray(minangle,c);
  c = _fromCharArray(angleincrement,c);
  c = _fromCharArray(maxrange,c);
  c = _fromCharArray(accuracy,c);

  return c;
}

char* ComboMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer;
  c = RobotMessage::toCharArray(c, bsize, skipHeader);
  c = VertexArrayMessage::toCharArray(c, bsize, true);
  c = RobotLaserMessage::toCharArray(c, bsize, true);
  return c;
}

const char* ComboMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  c = RobotMessage::fromCharArray(c, bsize, skipHeader);
  c = VertexArrayMessage::fromCharArray(c, bsize, true);
  c = RobotLaserMessage::fromCharArray(c, bsize, true);
  return c;
}

// serializes the message in the buffer
char* EdgeArrayMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer;
  size_t free = bsize;

  c = RobotMessage::toCharArray(c, free, skipHeader);
  free = (c) ? bsize - (c - buffer) : 0;

  size_t s =edgeVector.size();
  c = _toCharArray<size_t>(s,c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  for (size_t i =0; i< edgeVector.size(); i++){
    c = _toCharArray(edgeVector[i].idfrom, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].idto, c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].estimate[0], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].estimate[1], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].estimate[2], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].information[0], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].information[1], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].information[2], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].information[3], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].information[4], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
    c = _toCharArray(edgeVector[i].information[5], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
  }

  return c;
}

const char* EdgeArrayMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  c = RobotMessage::fromCharArray(c, bsize, skipHeader);
  size_t s;
  c = _fromCharArray(s,c);
  edgeVector.resize(s);
  for (size_t i =0; i< edgeVector.size(); i++){
    c = _fromCharArray(edgeVector[i].idfrom, c);
    c = _fromCharArray(edgeVector[i].idto, c);
    c = _fromCharArray(edgeVector[i].estimate[0], c);
    c = _fromCharArray(edgeVector[i].estimate[1], c);
    c = _fromCharArray(edgeVector[i].estimate[2], c);
    c = _fromCharArray(edgeVector[i].information[0], c);
    c = _fromCharArray(edgeVector[i].information[1], c);
    c = _fromCharArray(edgeVector[i].information[2], c);
    c = _fromCharArray(edgeVector[i].information[3], c);
    c = _fromCharArray(edgeVector[i].information[4], c);
    c = _fromCharArray(edgeVector[i].information[5], c);
  }
  return c;
}

// serializes the message in the buffer
char* ClosuresMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer;
  size_t free = bsize;

  c = RobotMessage::toCharArray(c, free, skipHeader);
  free = (c) ? bsize - (c - buffer) : 0;

  size_t s =closures.size();
  c = _toCharArray<size_t>(s,c, free);
  free = (c) ? bsize - (c - buffer) : 0;

  for (size_t i =0; i< closures.size(); i++){
    c = _toCharArray(closures[i], c, free);
    free = (c) ? bsize - (c - buffer) : 0;
  }

  return c;
}

const char* ClosuresMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  c = RobotMessage::fromCharArray(c, bsize, skipHeader);
  size_t s;
  c = _fromCharArray(s,c);
  closures.resize(s);
  for (size_t i =0; i< closures.size(); i++)
    c = _fromCharArray(closures[i], c);
  
  return c;
}

char* CondensedGraphMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer;
  c = RobotMessage::toCharArray(c, bsize, skipHeader);
  c = EdgeArrayMessage::toCharArray(c, bsize, true);
  c = ClosuresMessage::toCharArray(c, bsize, true);
  return c;
}

const char* CondensedGraphMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  c = RobotMessage::fromCharArray(c, bsize, skipHeader);
  c = EdgeArrayMessage::fromCharArray(c, bsize, true);
  c = ClosuresMessage::fromCharArray(c, bsize, true);
  return c;
}

char* GraphMessage::toCharArray(char* buffer, size_t bsize, bool skipHeader) const {
  char* c = buffer;
  size_t free = bsize;
  c = RobotMessage::toCharArray(c, bsize, skipHeader);
  free = (c) ? bsize - (c - buffer) : 0;
  c = VertexArrayMessage::toCharArray(c, free, true);
  free = (c) ? bsize - (c - buffer) : 0;
  c = EdgeArrayMessage::toCharArray(c, free, true);
  free = (c) ? bsize - (c - buffer) : 0;
  c = ClosuresMessage::toCharArray(c, free, true);
  return c;
}

const char* GraphMessage::fromCharArray(const char* buffer, size_t bsize, bool skipHeader) {
  const char* c = buffer;
  c = RobotMessage::fromCharArray(c, bsize, skipHeader);
  c = VertexArrayMessage::fromCharArray(c, bsize, true);
  c = EdgeArrayMessage::fromCharArray(c, bsize, true);
  c = ClosuresMessage::fromCharArray(c, bsize, true);
  return c;
}

RobotMessage* MessageFactory::constructMessage(int t) {
  std::map<int,BaseMessageCreator*>::iterator it = _msgCreatorMap.find(t);
  if (it==_msgCreatorMap.end())
    return 0;
  return (it->second)->constructMessage();
}

RobotMessage* MessageFactory::fromCharArray(const char* buf, size_t size) {
  assert (size > sizeof(int));
  int type;
  _fromCharArray(type, buf);
  const char* c = buf;
  RobotMessage* msg = constructMessage(type);
  cerr << "constructing a message of type" << type << endl;
  c = msg->fromCharArray(c);
  assert((c-buf)==size);
  return msg;
}
  

