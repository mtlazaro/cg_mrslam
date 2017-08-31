#pragma once

#include <iostream> 
#include <opencv2/highgui/highgui.hpp>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"

#include "geometry_msgs/Pose2D.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include "g2o/core/hyper_graph.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/data/robot_laser.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"

#include <opencv2/opencv.hpp>
#include "frequency_map.h"



using namespace std;
using namespace Eigen;
using namespace g2o;


class Graph2occupancy {
	
public:

	Graph2occupancy(OptimizableGraph *graph, cv::Mat *image,SE2 pose, float resolution = 0.05, float threhsold = 0.0, float rows = 0, float cols = 0, float maxRange = -1.0, float usableRange = -1.0, float gain = -1.0, float squareSize = 1.0, float angle = 0.0, float freeThrehsold = 0.0);

	void computeMap ();

	void setResolution (const float resolution);

	float getResolution ();
	float getThreshold ();
	float getRows ();
	float getCols ();
	float getFreeThreshold ();

	Vector2f getMapCenter();





protected:
	OptimizableGraph *_graph;
	FrequencyMap _map;
	cv::Mat *_mapImage;

	Vector2f _mapCenter;

	float _resolution;
	float _threshold;
	float _rows;
	float _cols;
	float _maxRange;
	float _usableRange;
	float _gain;
	float _squareSize;
	float _angle;
	float _freeThreshold;

	//Used for the occupancy map (published in RViz and provided via getMap)
	unsigned char _freeColor = 0;
	unsigned char _unknownColor = -1;
	unsigned char _occupiedColor = 100;


};


