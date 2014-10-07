#pragma once
#include "Particle.h"

#include <numeric>

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>

class MCFilter {
private:
	std::vector<Particle> _particles;
	std::vector<double> _weights;
	unsigned int _nParticles;
	Map * _map_ptr;

public:

	MCFilter(int nParticles, Map *map_ptr) :
			_nParticles(nParticles), _map_ptr(map_ptr) {

		_particles.reserve(_nParticles);
		_weights = std::vector<double>(_nParticles, 1.0 / _nParticles);
	}
	void init();
	void loop(LaserData, LaserData);
	void show_particles(Map* map_ptr, bool save = false, int seq = 0);
};

