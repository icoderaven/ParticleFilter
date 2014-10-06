#include "MCFilter.h"

void MCFilter::loop(LaserData prev_data, LaserData sensor_data) {
	double max_weight = -1;
	boost::uniform_real<> dist(0, 1.0 / _nParticles);
	boost::variate_generator<boost::mt19937&, boost::uniform_real<> > uni(_gen,
			dist);
	boost::uniform_int<> loc_dist(0, Particle::valid_locations.size());
	boost::uniform_int<> ang_dist(0, 180);	//One out of 180/n angles

	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > uni_loc(
			_gen, loc_dist);
	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > uni_ang(
			_gen, ang_dist);
	std::vector<Particle> propogated_particles, old_particles;
	propogated_particles.reserve(_nParticles);
	old_particles.reserve(_nParticles);
	double sum = 0.0;

	for (uint i = 0; i < _nParticles; i++) {
		/**
		 * Sample motion model
		 */
		Particle propogated = _particles[i].propogate(prev_data, sensor_data);
		while (propogated.getX() == -1 || _particles[i].getX() == -1) {
			int index_loc = uni_loc(), index_ang = uni_ang();
			cv::Point pt;
			try {
				 pt = Particle::valid_locations[index_loc];
			}

			catch (const std::out_of_range& oor) {
				std::cerr << "My 2nd Out of Range error: " << oor.what() << '\n';

			}
			_particles[i] = Particle(pt.x, pt.y, index_ang * M_PI / 180.0f,
					_map_ptr);
			propogated = _particles[i].propogate(prev_data, sensor_data);
		}
		propogated_particles[i] = propogated;
		old_particles.push_back(_particles[i]);
		/**
		 * Obtain measurement probability
		 */
		_weights[i] = propogated_particles[i].evaluate_measurement_probability(
				sensor_data, _nParticles);

		sum += _weights[i];
		if (_weights[i] >= max_weight) {
			max_weight = _weights[i];
		}
	}

	//See the propogated particles
//	cv::Mat temp;
//	cv::cvtColor(_map_ptr->get_map(), temp, CV_GRAY2BGR);
//	for (int i = 0; i < _nParticles; i++) {
//		cv::circle(temp, cv::Point(propogated_particles[i].getX(),propogated_particles[i].getY()), 1, CV_RGB(0,0,255));
//	}
//	cv::imshow("Particles", temp);
//	cv::waitKey(-1);

//Normalize weights
	std::cout << "\nWeights = ";
	for (uint i = 0; i < _nParticles; i++) {
		_weights[i] /= sum;
		std::cout << " " << _weights[i];
	}
	std::cout << "\n";
	/**
	 * Now draw M particles from this new weight distribution
	 * Using the algorithm for a low var sampler
	 */
	//Generate a CDF
	double cdf;
	cdf = _weights[0];

	int i = 0;
	//Draw uniform number between 0 and 1
//	std::cout<<"\nResampling...";
	double r = uni();
//	std::cout << "r = "<<r<<" c ="<<cdf<<"\n";
	for (uint m = 0; m < _nParticles; m++) {
		double U = r + (m) / (float) (_nParticles);
		while (U > cdf) {
			i = i + 1;
			cdf = cdf + _weights[i];
		}
//Sample this index particle again
//		std::cout<<i<<" "<<U<<" " <<cdf<<", "<<old_particles.size()<<"; \n";
		_particles[m] = old_particles.at(i).propogate(prev_data, sensor_data);
	}
//	std::cout << "\n";
}

void MCFilter::init() {
//If this is the first time we're starting the filter, randomly scatter particles
//But only scatter them in valid regions
	boost::uniform_int<> loc_dist(0, Particle::valid_locations.size());
	boost::uniform_int<> ang_dist(0, 180);	//One out of 180/n angles

	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > uni_loc(
			_gen, loc_dist);
	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > uni_ang(
			_gen, ang_dist);
	//And now sample from these
	for (int i = 0; i < _nParticles; i++) {
		int index_loc = uni_loc(), index_ang = uni_ang();
		cv::Point pt = Particle::valid_locations[index_loc];
		_particles[i] = Particle(pt.x, pt.y, index_ang * M_PI / 180.0f,
				_map_ptr);
	}
	show_particles(_map_ptr);
}

void MCFilter::show_particles(Map* map_ptr) {
	cv::Mat temp;
	cv::cvtColor(map_ptr->get_map(), temp, CV_GRAY2BGR);
	for (int i = 0; i < _nParticles; i++) {
		_particles[i].markParticle(&temp);
	}
	cv::imshow("Particles", temp);
	cv::waitKey(-1);
}
