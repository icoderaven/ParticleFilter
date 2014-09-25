#pragma once
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include "Matrix.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class Map {
private:
	cv::Mat _map;
	unsigned int _width, _height;
	float _min_x, _max_x, _min_y, _max_y;
	std::string _mapName;
public:
	Map(std::string mapName) :
			_mapName(mapName) {
		init(1,1);
	}

	void init(unsigned int width, unsigned int height){
		_width = width;
		_height = height;
		_map = cv::Mat(cv::Size(_width, _height), CV_32F);
		_min_x = _width;
		_max_x = 0;
		_min_y = _height;
		_max_y = 0;
		std::cout<<"\n[MapReader] Initialized map structure of dimensions "<<_height <<"x"<<_width<<"\n" ;
	}

	int read_file();
	cv::Mat get_map(){
		return _map;
	}

};
