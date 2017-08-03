#include "graph2occupancy.h"

using namespace std;
using namespace Eigen;
using namespace g2o;


Graph2occupancy::Graph2occupancy(OptimizableGraph *graph, cv::Mat *image, SE2 pose, float resolution, float threhsold, float rows, float cols, float maxRange, float usableRange, float gain, float squareSize, float angle, float freeThrehsold){
  
    _graph = graph;
    _mapImage = image;


    _resolution = resolution;
    _threshold = threhsold;
    _rows = rows;
    _cols = cols;

    if (usableRange > 12.0){
      usableRange = 12.0;
        }

    _initialGroundTruth = pose;

    _maxRange = maxRange;
    _usableRange = usableRange;
    _gain = gain;
    _squareSize = squareSize;
    _angle = angle;
    _freeThreshold = freeThrehsold;

}


void Graph2occupancy::computeMap(){

    // Sort verteces
    vector<int> vertexIds(_graph->vertices().size());
    int k = 0;
    for(OptimizableGraph::VertexIDMap::iterator it = _graph->vertices().begin(); it != _graph->vertices().end(); ++it) {
      vertexIds[k++] = (it->first);
    }  
    sort(vertexIds.begin(), vertexIds.end());


  /************************************************************************
   *                          Compute map size                            *
   ************************************************************************/
  // Check the entire graph to find map bounding box
  Matrix2d boundingBox = Matrix2d::Zero();
  std::vector<RobotLaser*> robotLasers;
  std::vector<SE2> robotPoses;
  double xmin=std::numeric_limits<double>::max();
  double xmax=std::numeric_limits<double>::min();
  double ymin=std::numeric_limits<double>::max();
  double ymax=std::numeric_limits<double>::min();

  SE2 baseTransform(0,0,_angle);

  for(size_t i = 0; i < vertexIds.size(); ++i) {
    OptimizableGraph::Vertex *_v = _graph->vertex(vertexIds[i]);
    VertexSE2 *v = dynamic_cast<VertexSE2*>(_v);
    if(!v) { continue; }
    v->setEstimate(baseTransform*v->estimate());
    OptimizableGraph::Data *d = v->userData();

    while(d) {
      RobotLaser *robotLaser = dynamic_cast<RobotLaser*>(d);
      if(!robotLaser) {
  d = d->next();
  continue;
      }      
      robotLasers.push_back(robotLaser);
      robotPoses.push_back(v->estimate());

      

      double x = v->estimate().translation().x();
      double y = v->estimate().translation().y();
      
      xmax = xmax > x+_maxRange ? xmax : x + _maxRange;
      ymax = ymax > y+_maxRange ? ymax : y + _maxRange;
      xmin = xmin < x-_maxRange ? xmin : x - _maxRange;
      ymin = ymin < y-_maxRange ? ymin : y - _maxRange;
 
      d = d->next();
    }
  }

  boundingBox(0,0)=xmin;
  boundingBox(1,0)=ymin;
  boundingBox(0,1)=xmax;
  boundingBox(1,1)=ymax;


  if(robotLasers.size() == 0)  {
    std::cout << "No laser scans found ... quitting!" << std::endl;
    return;
  }

  /************************************************************************
   *                          Compute the map                             *
   ************************************************************************/
  // Create the map
  Vector2i size;
  Vector2f offset;
  if(_rows != 0 && _cols != 0) { 
  	size = Vector2i(_rows, _cols); 
    offset = Vector2f(-size.x() * _resolution / 2.0f, -size.y() * _resolution / 2.0f);
    _mapCenter = offset;
							}
  else {
    size = Vector2i((boundingBox(0, 1) - boundingBox(0, 0))/ _resolution, (boundingBox(1, 1) - boundingBox(1, 0))/ _resolution);
   	offset = Vector2f(boundingBox(0, 0),boundingBox(1, 0));

   	_mapCenter = Vector2f(boundingBox(1, 0) - _initialGroundTruth.translation().y(),- boundingBox(0, 1) + _initialGroundTruth.translation().x());
    } 



  if(size.x() == 0 || size.y() == 0) {
    std::cout << "Zero map size ... quitting!" << std::endl;
   return;
  }





  FrequencyMapCell unknownCell;
  
  _map = FrequencyMap(_resolution, offset, size, unknownCell);

  for(size_t i = 0; i < vertexIds.size(); ++i) {
    OptimizableGraph::Vertex *_v = _graph->vertex(vertexIds[i]);
    VertexSE2 *v = dynamic_cast<VertexSE2*>(_v);
    if(!v) { continue; }
    OptimizableGraph::Data *d = v->userData();
    SE2 robotPose = v->estimate();
    
    while(d) {
      RobotLaser *robotLaser = dynamic_cast<RobotLaser*>(d);
      if(!robotLaser) {
  d = d->next();
  continue;
      }      
      _map.integrateScan(robotLaser, robotPose, _maxRange, _usableRange, _gain, _squareSize);
      d = d->next();
    }
  }



  /************************************************************************
   *                  Convert frequency map into int[8]                   *
   ************************************************************************/

  *_mapImage = cv::Mat(_map.rows(), _map.cols(), CV_8UC1);
  _mapImage->setTo(cv::Scalar(0));

    for(int c = 0; c < _map.cols(); c++) {
      for(int r = 0; r < _map.rows(); r++) {
          if(_map(r, c).misses() == 0 && _map(r, c).hits() == 0) {
            _mapImage->at<unsigned char>(r, c) = _unknownColor; }
          else {
            float fraction = (float)_map(r, c).hits()/(float)(_map(r, c).hits()+_map(r, c).misses());
            if (_freeThreshold && fraction < _freeThreshold){
              _mapImage->at<unsigned char>(r, c) = _freeColor; }
            else if (_threshold && fraction > _threshold){
              _mapImage->at<unsigned char>(r, c) = _occupiedColor; }
            else {
            _mapImage->at<unsigned char>(r, c) = _unknownColor; }
              }
          
          }
      }



}





  void Graph2occupancy::setResolution (const float resolution){
    _resolution = resolution;
  }

  float Graph2occupancy::getResolution (){
    return _resolution;
  }
  float Graph2occupancy::getThreshold (){
    return _threshold;
  }
  float Graph2occupancy::getRows (){
    return _rows;
  }
  float Graph2occupancy::getCols (){
    return _cols;
  }
  float Graph2occupancy::getFreeThreshold (){
    return _freeThreshold;
  }


  Vector2f Graph2occupancy::getMapCenter(){
    return _mapCenter;
  }





