#include "MCFilter.h"

void MCFilter::loop(LaserData prev_data, LaserData sensor_data) {
	double max_weight = -1;
	boost::uniform_real<> dist(0, 1);
	boost::variate_generator<boost::mt19937&, boost::uniform_real<> > uni(_gen,
			dist);

	std::vector<Particle> propogated_particles;
	propogated_particles.reserve(_nParticles);
	double sum = 0.0;
	for (uint i = 0; i < _nParticles; i++) {
		/**
		 * Sample motion model
		 */
		propogated_particles[i] = _particles[i].propogate(prev_data,
				sensor_data);
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
	std::cout<<"\nWeights = ";
	for (uint i = 0; i < _nParticles; i++) {
		_weights[i] /= sum;
		std::cout<<" "<<_weights[i];
	}
	std::cout<<"\n";
	/**
	 * Now draw M particles from this new weight distribution
	 * Using the algorithm for a low var sampler
	 */
	//Generate a CDF
	double cdf;
	cdf = _weights[0];
	std::cout<<"\n";
	int i = 0;
	//Draw uniform number between 0 and 1
	double r = uni();
	for (uint m = 0; m < _nParticles; m++) {
		double U = r + (m )/(_nParticles);
		while (U>cdf)
		{
			i = i+1;
			cdf = cdf + _weights[i];
		}
		//Sample this index particle again
		_particles[i] = propogated_particles[i];
	}
}

void MCFilter::init(Map *map_ptr) {
	//If this is the first time we're starting the filter, randomly scatter particles
	//But only scatter them in valid regions

	//Find all points that are not an obstacle
	cv::Mat possible_locations = map_ptr->get_map();
//	std::cout<<possible_locations.rows<<"."<<possible_locations.cols<<"\n";
//	cv::imshow("Valid regions", possible_locations);
//	cv::waitKey(-1);
	std::vector<cv::Point> valid_locations;

	for (int i = 0; i < possible_locations.rows; i++) {
		for (int j = 0; j < possible_locations.cols; j++) {
			if (possible_locations.at<float>(i, j) == 0) {
				valid_locations.push_back(cv::Point(j, i));
			}
		}
	}
	boost::uniform_int<> loc_dist(0, valid_locations.size());
	boost::uniform_int<> ang_dist(0, 180);	//One out of 180 angles

	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > uni_loc(
			_gen, loc_dist);
	boost::variate_generator<boost::mt19937&, boost::uniform_int<> > uni_ang(
			_gen, ang_dist);
	//And now sample from these
	for (int i = 0; i < _nParticles; i++) {
		int index_loc = uni_loc(), index_ang = uni_ang();
		cv::Point pt = valid_locations[index_loc];
		_particles[i] = Particle(pt.x, pt.y, index_ang * M_PI / 180.0f,
				map_ptr);
	}
	show_particles(map_ptr);
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
