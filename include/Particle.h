#pragma once
#include <math.h>
#include "Map.h"
#include "LogReader.h"
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/math/distributions/normal.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include <stdexcept>

class Particle {
private:
	float _x, _y, _theta;
	Map *_map_ptr;
	static float _dist_max, _sigma_sensor, _sigma_sample[3];
	static float _alpha[4];
	static float _z_hit, _z_unif;
	static std::vector<std::vector<float> > _precomputed_distances;
	boost::mt19937 _gen;
public:
	static std::vector<cv::Point> valid_locations;
	static cv::Mat valid_locations_map;

	Particle(float x, float y, float theta, Map *map_ptr) :
			_x(x), _y(y), _theta(theta), _map_ptr(map_ptr) {
	}
	Particle(const Particle &p) {
		_x = p.getX();
		_y = p.getY();
		_theta = p.getTheta();
		_map_ptr = p.getMapPtr();
	}
	Particle & operator=(const Particle & p) {
		_x = p.getX();
		_y = p.getY();
		_theta = p.getTheta();
		_map_ptr = p.getMapPtr();
		return *this;
	}
	Particle propogate(LaserData, LaserData);
	double evaluate_measurement_probability(LaserData, float);
	double gaussian_prob(float, float, float);
	double sample_gaussian(float, float);
	float wrap_pi(float);
	void markParticle(cv::Mat *);

	Map* getMapPtr() const {
		return _map_ptr;
	}

	float getTheta() const {
		return _theta;
	}

	float getX() const {
		return _x;
	}

	float getY() const {
		return _y;
	}

	static void determine_valid_locations(Map*);
};
