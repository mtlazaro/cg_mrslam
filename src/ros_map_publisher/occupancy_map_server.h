#pragma once

#include <iostream> 
#include <fstream>

#include "ros/ros.h"
#include <ros/package.h>

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"

#include "geometry_msgs/Pose2D.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"


#include "g2o/types/slam2d/vertex_se2.h"

#include <boost/filesystem.hpp> 

#include <opencv2/opencv.hpp>

using namespace std;
using namespace Eigen;
using namespace g2o;

class OccupancyMapServer{

public:

	OccupancyMapServer(cv::Mat* occupancyMap, int idRobot, string mapFrameName = "map", string mapTopicName = "map", float threshold = 0.0, float freeThreshold = 0.0);

	void publishMap ();
	void publishMapMetaData();

	bool mapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res);

	void saveMap(string outputFileName);

	void setOffset(Vector2f offset);

	void setResolution(float resolution);




protected:


	cv::Mat * _occupancyMap;
	cv::Mat _occupancyMapImage;

	string _mapTopicName;
	string _mapFrameName;

	Vector2f _mapOffset;
	float _mapResolution;

	int _idRobot;

	//Used for the occupancy map (published in RViz and provided via getMap)
	unsigned char _freeColor = 0;
	unsigned char _unknownColor = -1;
	unsigned char _occupiedColor = 100;

	//Used for colouring the image saved on disk
	unsigned char _freeImageColor = 255;
	unsigned char _unknownImageColor = 127;
	unsigned char _occupiedImageColor = 0;

	float _threshold;
	float _freeThreshold;

	ros::NodeHandle _nh;
	ros::Publisher _pubOccupGrid;
	ros::Publisher _pubMapMetaData;
	ros::ServiceServer _server;

	nav_msgs::OccupancyGrid _gridMsg;
	nav_msgs::GetMap::Response _resp;


};




