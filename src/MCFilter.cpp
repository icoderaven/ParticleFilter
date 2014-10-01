#include "MCFilter.h"

void MCFilter::loop(LaserData sensor_data) {
	double max_weight = -1;
	boost::uniform_real<> dist(0, 1);
	boost::mt19937 gen;
	boost::variate_generator<boost::mt19937&, boost::uniform_real<> > uni(gen,
			dist);

	std::vector<Particle> propogated_particles;
	propogated_particles.reserve(_nParticles);
	double sum = 0.0;
	for (uint i = 0; i < _nParticles; i++) {
		/**
		 * Sample motion model
		 */

		/**
		 * Obtain measurement probability
		 */
		_weights[i] = propogated_particles[i].evaluate_measurement_probability(
				sensor_data);
		sum += _weights[i];
		if (_weights[i] >= max_weight) {
			max_weight = _weights[i];
		}

	}
	//Normalize weights
	for (uint i = 1; i < _nParticles; i++) {
		_weights[i] /= sum;
	}

	/**
	 * Now draw M particles from this new weight distribution
	 */
	//Generate a CDF
	std::vector<double> cdf(_nParticles);
	cdf[0] = _weights[0];
	for (uint i = 1; i < _nParticles; i++) {
		cdf[i] = cdf[i - 1] + _weights[i];
	}
	cdf[cdf.size() - 1] = 1.0;
	for (uint i = 0; i < _nParticles; i++) {
		//Draw uniform number between 0 and 1
		std::vector<double>::iterator pos;
		pos = std::lower_bound(cdf.begin(), cdf.end(), uni());
		//Sample this index particle again
		_new_particles[i] = propogated_particles[pos - cdf.begin()];
	}
}
