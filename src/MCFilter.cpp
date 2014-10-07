#include "MCFilter.h"

void MCFilter::loop(LaserData prev_data, LaserData sensor_data) {
	double max_weight = -1;
	boost::uniform_real<> dist(0, 1.0 / _nParticles);
	boost::variate_generator<boost::mt19937&, boost::uniform_real<> > uni(
			Map::_gen, dist);
	boost::uniform_int<> loc_dist(0, Particle::valid_locations.size());
	boost::uniform_int<> ang_dist(0, 180);	//One out of 180/n angles

	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > uni_loc(
			Map::_gen, loc_dist);
	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > uni_ang(
			Map::_gen, ang_dist);
	std::vector<Particle> propogated_particles;
	std::vector<Particle> old_particles;
	propogated_particles.resize(_nParticles, _particles[0]);
	old_particles.resize(_nParticles, _particles[0]);
	double sum = 0.0;

#pragma omp parallel for
	for (uint i = 0; i < _nParticles; i++) {
		/**
		 * Sample motion model
		 */
		Particle propogated = _particles[i].propogate(prev_data, sensor_data);
//		while (propogated.getX() == -1 || _particles[i].getX() == -1) {
//			int index_loc = uni_loc(), index_ang = uni_ang();
//			cv::Point pt;
//			try {
//				 pt = Particle::valid_locations[index_loc];
//			}
//
//			catch (const std::out_of_range& oor) {
//				std::cerr << "My 2nd Out of Range error: " << oor.what() << '\n';
//
//			}
//			_particles[i] = Particle(pt.x, pt.y, index_ang * M_PI / 180.0f,
//					_map_ptr);
//			propogated = _particles[i].propogate(prev_data, sensor_data);
//		}
		propogated_particles[i] = propogated;
		old_particles[i] = _particles[i];
		/**
		 * Obtain measurement probability
		 */
		if (propogated.getX() == -1 || _particles[i].getX() == -1) {
			_weights[i] = 0.0;
		} else {
			_weights[i] =
					propogated_particles[i].evaluate_measurement_probability(
							sensor_data, _nParticles);
		}
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
//	std::cout << "\nWeights = ";
	for (uint i = 0; i < _nParticles; i++) {
		_weights[i] /= sum;
//		std::cout << " " << _weights[i];
	}
//	std::cout << "\n";
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
	int last_index = -1;
//	std::cout << "r = "<<r<<" c ="<<cdf<<"\n";
	for (uint m = 0; m < _nParticles; m++) {
		double U = r + (m) / (float) (_nParticles);
		while (U > cdf) {
			i = i + 1;
			cdf = cdf + _weights[i];
		}
//Sample this index particle again
//		std::cout<<i<<" "<<U<<" " <<cdf<<", "<<old_particles.size()<<", "<<propogated_particles.size()<<"; \n";
//		_particles[m] = old_particles.at(i).propogate(prev_data, sensor_data);
		if (last_index == i) {
			_particles[m] = propogated_particles.at(i).perturb();
		} else {
			_particles[m] = propogated_particles.at(i);
		}
		last_index = i;
	}
//	std::cout << "\n";
}

void MCFilter::init() {
//If this is the first time we're starting the filter, randomly scatter particles
//But only scatter them in valid regions
	boost::uniform_int<> loc_dist(0, Particle::valid_locations.size());
	boost::uniform_int<> ang_dist(0, 180);	//One out of 180/n angles

	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > uni_loc(
			Map::_gen, loc_dist);
	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > uni_ang(
			Map::_gen, ang_dist);
	//And now sample from these
	for (int i = 0; i < _nParticles; i++) {
		int index_loc = uni_loc(), index_ang = uni_ang();
		cv::Point pt = Particle::valid_locations[index_loc];
		_particles[i] = Particle(pt.x, pt.y, index_ang * M_PI / 180.0f,
				_map_ptr);
	}
	show_particles(_map_ptr);
}

void MCFilter::show_particles(Map* map_ptr, bool save , int seq) {
	cv::Mat temp(map_ptr->get_map().size(), CV_8UC3);
	cv::cvtColor(map_ptr->get_map(), temp, CV_GRAY2RGB);
	temp *=255;
	for (int i = 0; i < _nParticles; i++) {
		_particles[i].markParticle(&temp);
	}
	cv::imshow("Particles", temp);
	std::vector<int> compression_params;
	    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	    compression_params.push_back(3);
	if (save) {
		std::ostringstream s;
		s << "/home/icoderaven/temp/img_" << seq << ".png";
		cv::imwrite(s.str(), temp,compression_params);
	}
	cv::waitKey(100);
}
