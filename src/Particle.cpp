#include "Particle.h"
float Particle::_dist_max = 500.0;
float Particle::_sigma_sensor = 5.0;
float Particle::_sigma_sample[3] = { 0.01, 1, 0.01 };
float Particle::_alpha[4] = { 0.05, 0, 0.1, 1 };
std::vector<cv::Point> Particle::valid_locations;
std::vector<std::vector<float> > Particle::_precomputed_distances(
		std::vector<float>(360));

/**
 * p(x_t | x_{t-1}, u_{t-1})
 */
Particle Particle::propogate(LaserData prev_data, LaserData new_data) {
	//Move forward with some random disturbance
	float delta_rot1 = atan2(new_data.getY() - prev_data.getY(),
			new_data.getX() - prev_data.getX()) - prev_data.getTheta();
	float delta_trans = sqrt(
			pow(new_data.getX() - prev_data.getX(), 2)
					+ pow(new_data.getY() - prev_data.getY(), 2));
	float delta_rot2 = new_data.getTheta() - prev_data.getTheta() - delta_rot1;

	delta_rot1 = wrap_pi(delta_rot1);
	delta_rot2 = wrap_pi(delta_rot2);
//
//	std::cout << "\nD " << delta_rot1 << ", " << delta_trans << ", "
//			<< delta_rot2 << "\n";
	float sampled_delta_rot1 = wrap_pi(
			delta_rot1
					- (0, sample_gaussian(0,
							_alpha[0] * fabs(delta_rot1)
									+ _alpha[1] * delta_trans)));
	float sampled_delta_trans = delta_trans
			- sample_gaussian(0,
					_alpha[2] * delta_trans + _alpha[3] * fabs(delta_rot1)
							+ fabs(delta_rot2));
	float sampled_delta_rot2 = wrap_pi(
			delta_rot2
					- (sample_gaussian(0,
							_alpha[0] * fabs(delta_rot2)
									+ _alpha[1] * delta_trans)));

//	std::cout << sampled_delta_rot1 << ", " << sampled_delta_trans << ", "
//			<< sampled_delta_rot2 << "\n";
	float x, y, theta;
	x = _x + sampled_delta_trans * cos((_theta + sampled_delta_rot1));
	y = _y + sampled_delta_trans * sin((_theta + sampled_delta_rot1));
	theta = wrap_pi(_theta + sampled_delta_rot1 + sampled_delta_rot2);
//	std::cout<<x<<", "<<y<<", "<<theta<<"\n";
	return Particle(x, y, theta, _map_ptr);
}

float Particle::wrap_pi(float angle) {
	if (angle > 0)
		angle = fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
	else
		angle = fmod(angle - M_PI, 2.0 * M_PI) + M_PI;
	return angle;
}
/**
 * p( z_t | x_t, m)
 */
double Particle::evaluate_measurement_probability(LaserData sensor_data,
		float M) {
	//Ray trace from current position
	cv::Mat image;
	cv::cvtColor(_map_ptr->get_map(), image, CV_GRAY2BGR);
	//Later do caching
	double probabilities[180];
	//For each angle in the local frame,
//	std::cout << "Entering the lair" << _map_ptr->get_map().rows << ","
//			<< _map_ptr->get_map().cols << "\n";
	for (int i = 0; i < 180; i++) {
		//move outwards till you intersect an object...

		//This algorithm is apparently called DDA : http://lodev.org/cgtutor/raycasting.html

		//Determine ray direction
		float angle = (_theta + (i - 90.0) * M_PI / 180.0f);
		float rayDirX = cos(angle);
		float rayDirY = sin(angle);

//		std::cout<<"Entering the angle "<<angle*180.0f/M_PI<<"\n";
		//which box of the map we're in
		int mapX = int(_x);
		int mapY = int(_y);

		//length of ray from current position to next x or y-side
		double sideDistX;
		double sideDistY;

		//length of ray from one x or y-side to next x or y-side
		double deltaDistX = sqrt(1 + (rayDirY * rayDirY) / (rayDirX * rayDirX));
		double deltaDistY = sqrt(1 + (rayDirX * rayDirX) / (rayDirY * rayDirY));

		//what direction to step in x or y-direction (either +1 or -1)
		int stepX;
		int stepY;

		int hit = 0; //was there a wall hit?
		int side; //was a NS or a EW wall hit?

		//calculate step and initial sideDist
		if (rayDirX < 0) {
			stepX = -1;
			sideDistX = (_x - mapX) * deltaDistX;
		} else {
			stepX = 1;
			sideDistX = (mapX + 1.0 - _x) * deltaDistX;
		}
		if (rayDirY < 0) {
			stepY = -1;
			sideDistY = (_y - mapY) * deltaDistY;
		} else {
			stepY = 1;
			sideDistY = (mapY + 1.0 - _y) * deltaDistY;
		}

		//perform DDA
//		std::cout<<"Starting DDA\n";
		while (hit == 0) {
			//jump to next map square, OR in x-direction, OR in y-direction
			if (sideDistX < sideDistY) {
				sideDistX += deltaDistX;
				mapX += stepX;
				side = 0;
			} else {
				sideDistY += deltaDistY;
				mapY += stepY;
				side = 1;
			}
//			cv::circle(image, cv::Point(mapX, mapY), 1, CV_RGB(0,255,0));
//			std::cout<<"Check! "<<mapX<<", "<<mapY<<", "<<_map_ptr->get_map().at<float>(mapX, mapY)<<" \n";
//			cv::imshow("Debug", image);
//			cv::waitKey(1);
			//Check if ray has hit a wall
//			std::cout<<"Check! "<<mapX<<", "<<mapY<<" \n";
			//Bounds check
			if ((mapX >= 0 && mapX < _map_ptr->get_map().cols)
					&& (mapY >= 0 && mapY < _map_ptr->get_map().rows)) {
				if (_map_ptr->get_map().at<float>(mapY, mapX) > 0.5) //Ugh, indexing!
					hit = 1;
			} else {
//				std::cout << "Max Range! " << mapX << ", " << mapY << " \n";
				break;
			}
		}
//		std::cout<<"DDA done\n i = "<<i<<"\n";
		//Find the distance
		double dist;
		if (hit) {
			dist = sqrt((mapX - _x) * (mapX - _x) + (mapY - _y) * (mapY - _y));
		} else {
			//@todo: specify this in a more smart manner
			dist = _dist_max;
		}

		//Now construct fancy probability distribution here
		//@todo: learn these parameters?
		float z_hit = 1.0, z_unif = 1.0;
		//@todo: Sensor reading goes here
		probabilities[i] = z_hit
				* gaussian_prob(sensor_data.getRanges()[i], dist,
						_sigma_sensor);
//		std::cout << dist << " = " << probabilities[i] << ", "
//				<< sensor_data.getRanges()[i] << "\n";
//		std::cout<<"Survival!\n";
		//DEBUG:: plotting these lines!!
		cv::line(image, cv::Point(_x, _y), cv::Point(mapX, mapY),
				CV_RGB(255,0,0));
	}
	double q = 0.0;
	for (int i = 0; i < 180; i++) {
		q += std::isinf(log(probabilities[i])) ? 0 : log(probabilities[i]);
//		std::cout<<"\n"<<log(probabilities[i]);
	}
//	cv::imshow("Debug", image);
//	cv::waitKey(1);
//	std::cout << "\nq = "<<q << "\n";
	return exp((1 / M) * q);
}

double Particle::gaussian_prob(float query_val, float mean_dist,
		float std_dev) {
	boost::math::normal_distribution<double> zhit_prob(mean_dist, std_dev);
	return pdf(zhit_prob, query_val)
			/ (cdf(zhit_prob, _dist_max) - cdf(zhit_prob, 0));
}

double Particle::sample_gaussian(float mean_dist, float std_dev) {
	boost::normal_distribution<double> dist(mean_dist, std_dev);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<double> > normal(
			_gen, dist);
	return normal();
}

void Particle::markParticle(cv::Mat *image) {
	cv::circle(*image, cv::Point(_x, _y), 1, CV_RGB(255,0,0));
}

void Particle::determine_valid_locations(Map * map_ptr) {
	//Find all points that are not an obstacle
	cv::Mat possible_locations = map_ptr->get_map();
//	std::cout<<possible_locations.rows<<"."<<possible_locations.cols<<"\n";
//	cv::imshow("Valid regions", possible_locations);
//	cv::waitKey(-1);
	for (int i = 0; i < possible_locations.rows; i++) {
		for (int j = 0; j < possible_locations.cols; j++) {
			if (possible_locations.at<float>(i, j) == 0) {
				valid_locations.push_back(cv::Point(j, i));
			}
		}
	}
	for (int i = 0; i < valid_locations.size(); i++) {
		for (int j = 0; j < 360; i++) {
			//move outwards till you intersect an object...

			//This algorithm is apparently called DDA : http://lodev.org/cgtutor/raycasting.html

			//Determine ray direction
			float angle = j * M_PI / 180.0f);
			float rayDirX = cos(angle);
			float rayDirY = sin(angle);

			//		std::cout<<"Entering the angle "<<angle*180.0f/M_PI<<"\n";
			//which box of the map we're in
			int mapX = valid_locations[i].x;
			int mapY = valid_locations[i].y;

			//length of ray from current position to next x or y-side
			double sideDistX;
			double sideDistY;

			//length of ray from one x or y-side to next x or y-side
			double deltaDistX = sqrt(
					1 + (rayDirY * rayDirY) / (rayDirX * rayDirX));
			double deltaDistY = sqrt(
					1 + (rayDirX * rayDirX) / (rayDirY * rayDirY));

			//what direction to step in x or y-direction (either +1 or -1)
			int stepX;
			int stepY;

			int hit = 0; //was there a wall hit?
			int side; //was a NS or a EW wall hit?

			//calculate step and initial sideDist
			if (rayDirX < 0) {
				stepX = -1;
				sideDistX = (_x - mapX) * deltaDistX;
			} else {
				stepX = 1;
				sideDistX = (mapX + 1.0 - _x) * deltaDistX;
			}
			if (rayDirY < 0) {
				stepY = -1;
				sideDistY = (_y - mapY) * deltaDistY;
			} else {
				stepY = 1;
				sideDistY = (mapY + 1.0 - _y) * deltaDistY;
			}

			//perform DDA
			//		std::cout<<"Starting DDA\n";
			while (hit == 0) {
				//jump to next map square, OR in x-direction, OR in y-direction
				if (sideDistX < sideDistY) {
					sideDistX += deltaDistX;
					mapX += stepX;
					side = 0;
				} else {
					sideDistY += deltaDistY;
					mapY += stepY;
					side = 1;
				}
				//			cv::circle(image, cv::Point(mapX, mapY), 1, CV_RGB(0,255,0));
				//			std::cout<<"Check! "<<mapX<<", "<<mapY<<", "<<_map_ptr->get_map().at<float>(mapX, mapY)<<" \n";
				//			cv::imshow("Debug", image);
				//			cv::waitKey(1);
				//Check if ray has hit a wall
				//			std::cout<<"Check! "<<mapX<<", "<<mapY<<" \n";
				//Bounds check
				if ((mapX >= 0 && mapX < map_ptr->get_map().cols)
						&& (mapY >= 0 && mapY < map_ptr->get_map().rows)) {
					if (map_ptr->get_map().at<float>(mapY, mapX) > 0.5) //Ugh, indexing!
						hit = 1;
				} else {
					//				std::cout << "Max Range! " << mapX << ", " << mapY << " \n";
					break;
				}
			}
			//		std::cout<<"DDA done\n i = "<<i<<"\n";
			//Find the distance
			double dist;
			if (hit) {
				dist = sqrt(
						pow((mapX - valid_locations[i].x), 2) + pow(mapY - valid_locations[i].y, 2));
			} else {
				dist = _dist_max;
			}

			_precomputed_distances[i][j] = dist;
		}
	}
}
