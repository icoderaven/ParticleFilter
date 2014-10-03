#pragma once
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <math.h>
//For transformations
#include "Eigen/Geometry"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
class OdometryData {
private:
	float _x, _y;
	float _theta;
	float _ts;
public:
	OdometryData(float x, float y, float theta, float ts) :
			_x(x), _y(y), _theta(theta), _ts(ts) {
	}
};

class LaserData {
private:
	float _x, _y, _theta;
	float _xl, _yl, _thetal;
	float _ranges[180];
	float _ts;

public:
	LaserData(float x, float y, float theta, float xl, float yl, float thetal,
			float ranges[], float ts) :
			_x(x), _y(y), _theta(theta), _xl(xl), _yl(yl), _thetal(thetal), _ts(
					ts) {
		for (int i = 0; i < 180; i++) {
			_ranges[i] = ranges[i];
		}
	}
	Eigen::Matrix3Xd sensed_locations();
	float getX() {
		return _x;
	}

	const float* getRanges() const {
		return _ranges;
	}

	float getTheta() const {
		return _theta;
	}

	float getThetal() const {
		return _thetal;
	}

	float getTs() const {
		return _ts;
	}

	float getXl() const {
		return _xl;
	}

	float getY() const {
		return _y;
	}

	float getYl() const {
		return _yl;
	}
};

class LogReader {
public:
	std::vector<OdometryData> _odoms;
	std::vector<LaserData> _lasers;
	std::string _logName;
public:
	LogReader(std::string logName) :
			_logName(logName) {
	}

	int read_file();
};
