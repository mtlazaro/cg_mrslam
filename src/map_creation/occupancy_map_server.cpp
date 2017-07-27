#include "occupancy_map_server.h"

using namespace std;

using namespace boost::filesystem;

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))


bool OccupancyMapServer::mapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res ){


  //header (uint32 seq, time stamp, string frame_id)
 res.map.header.frame_id = _mapTopicName;
  
  //info (time map_load_time  float32 resolution   uint32 width  uint32 height   geometry_msgs/Pose origin)

  res.map.info = _gridMsg.info;
  res.map.info.map_load_time = ros::Time::now();


  //data (int8[] data)
  res.map.data = _gridMsg.data;

  return true;
}


OccupancyMapServer::OccupancyMapServer(cv::Mat* occupancyMap, int idRobot, int typeExperiment, string mapTopicName, float threshold, float freeThreshold){

	_occupancyMap = occupancyMap;

	_typeExperiment = typeExperiment;
	_idRobot = idRobot;

	_threshold = threshold;
	_freeThreshold = freeThreshold;

	_mapTopicName = mapTopicName;

	_pubOccupGrid = _nh.advertise<nav_msgs::OccupancyGrid>(_mapTopicName,1);

	_pubMapMetaData = _nh.advertise<nav_msgs::MapMetaData>(_mapTopicName + "_metadata", 1);

	_server = _nh.advertiseService(_mapTopicName, &OccupancyMapServer::mapCallback, this);


	_gridMsg.header.frame_id = _mapTopicName;
	geometry_msgs::Pose poseMsg;
	poseMsg.position.x = 0.0;
	poseMsg.position.y = 0.0;
	poseMsg.position.z = 0.0;
	poseMsg.orientation.x = 0;
	poseMsg.orientation.y = 0; 
	poseMsg.orientation.z = 0; 
	poseMsg.orientation.w = 1.0;

	_gridMsg.info.origin = poseMsg;

}





void OccupancyMapServer::publishMap() {


	_gridMsg.data.resize(_occupancyMap->rows * _occupancyMap->cols);


	for(int r = 0; r < _occupancyMap->rows; r++) {
		for(int c = 0; c < _occupancyMap->cols; c++) {
			_gridMsg.data[MAP_IDX(_occupancyMap->cols, c, _occupancyMap->rows - r - 1)] = _occupancyMap->at<unsigned char>(r,c);

	}
	}


	//header (uint32 seq, time stamp, string frame_id)

	//info (time map_load_time  float32 resolution   uint32 width  uint32 height   geometry_msgs/Pose origin)



	_gridMsg.info.width = _occupancyMap->cols;
	_gridMsg.info.height = _occupancyMap->rows;

	_gridMsg.info.origin.position.x = _mapOffset.x();
	_gridMsg.info.origin.position.y = _mapOffset.y();

	_gridMsg.info.map_load_time = ros::Time::now();
	_gridMsg.info.resolution = _mapResolution;

	_pubOccupGrid.publish(_gridMsg);




}


void OccupancyMapServer::publishMapMetaData() {


	nav_msgs::MapMetaData msg;

	msg.resolution = _mapResolution;
	
	msg.origin.position.x = _mapOffset.x();
	msg.origin.position.y = _mapOffset.y();
	msg.origin.position.z = 0.0;
	msg.origin.orientation.x = 0;
	msg.origin.orientation.y = 0; 
	msg.origin.orientation.z = 0.0; 
	msg.origin.orientation.w = 1.0;

	_pubMapMetaData.publish(msg);


}




void OccupancyMapServer::saveMap(std::string outputFileName) {

	//Save files in the current working directory. Be careful if using ROSLAUNCH since the cwd becomes ~/.ros
	std::stringstream titleImage;
	titleImage <<outputFileName <<".png"; 

	std::stringstream titleYaml;
	titleYaml<<outputFileName <<".yaml"; 


	_occupancyMapImage = cv::Mat(_occupancyMap->rows, _occupancyMap->cols, CV_8UC1);
	_occupancyMapImage.setTo(cv::Scalar(0));


	for(int r = 0; r < _occupancyMap->rows; r++) {
		for(int c = 0; c < _occupancyMap->cols; c++) {
			if(_occupancyMap->at<unsigned char>(r, c) == _unknownColor) {
				_occupancyMapImage.at<unsigned char>(r, c) = _unknownImageColor;  }
			else if (_occupancyMap->at<unsigned char>(r, c) == _freeColor){
				_occupancyMapImage.at<unsigned char>(r, c) = _freeImageColor;   }
			else if (_occupancyMap->at<unsigned char>(r, c) == _occupiedColor){
				_occupancyMapImage.at<unsigned char>(r, c) = _occupiedImageColor;   }

		}
	} 


	cv::imwrite(titleImage.str(), _occupancyMapImage);


	std::ofstream ofs;
	ofs.open(titleYaml.str());
	ofs << "image: " << titleImage.str() << endl;
	ofs<< "resolution: " << _mapResolution << endl;
	ofs<< "origin: [" << _mapOffset.x() << ", " << _mapOffset.y() << ", " << 0.0 << "]" << endl;
	ofs<< "negate: 0" << endl;
	ofs<< "occupied_thresh: " << _threshold << endl;
	ofs<< "free_thresh: " << _freeThreshold << endl;

	ofs.close();

}







void OccupancyMapServer::setOffset(Vector2f offset){
	_mapOffset = offset;
}

void OccupancyMapServer::setResolution(float resolution){
	_mapResolution = resolution;
}


