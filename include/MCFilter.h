#pragma once
#include "Particle.h"

#include <numeric>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>


class MCFilter {
private:
	std::vector<Particle> _past_particles, _new_particles;
	std::vector<double> _weights;
	unsigned int _nParticles;
public:
	MCFilter(int nParticles) :
			_nParticles(nParticles) {

		_new_particles.reserve(_nParticles);
		_past_particles.reserve(_nParticles);
		_weights = std::vector<double>(_nParticles, 1.0 / _nParticles);
	}
	void init(Map*);
	void loop(LaserData, LaserData);
};

