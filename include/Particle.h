#pragma once
#include <math.h>
#include "Map.h"
#include "LogReader.h"
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/math/distributions/normal.hpp>
class Particle {
private:
	float _x, _y, _theta;
	Map *_map_ptr;
	static float _dist_max,_sigma;
public:
	Particle(float x, float y, float theta, Map *map_ptr) :
			_x(x), _y(y), _theta(theta), _map_ptr(map_ptr) {}
	void propogate();
	float evaluate_measurement_probability(LaserData sensor_data);
	float gaussian_prob(float, float, float);
};
