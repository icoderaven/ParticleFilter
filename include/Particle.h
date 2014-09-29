#pragma once
#include <math.h>
#include "Map.h"
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <opencv2/imgproc/imgproc.hpp>
class Particle {
private:
	float _x, _y, _theta;
	Map *_map_ptr;
	float dist_max;
public:
	Particle(float x, float y, float theta, Map *map_ptr) :
			_x(x), _y(y), _theta(theta), _map_ptr(map_ptr) {
		dist_max = 300;
	}
	void propogate();
	float evaluate_measurement_probability();
	float gaussian_spread(float, float);
};
