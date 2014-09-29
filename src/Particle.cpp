#include "Particle.h"

/**
 * p(x_t | x_{t-1}, u_{t-1})
 */
void Particle::propogate() {
	//Move forward with some random disturbance
}

/**
 * p( z_t | x_t, m)
 */
float Particle::evaluate_measurement_probability() {
	//Ray trace from current position
	cv::Mat image;
	cv::cvtColor(_map_ptr->get_map(), image, CV_GRAY2BGR);
	//Later do caching
	float probabilities[180];
	//For each angle in the local frame,
	std::cout << "Entering the lair" << _map_ptr->get_map().rows << ","
			<< _map_ptr->get_map().cols << "\n";
	for (int i = 0; i < 180; i++) {
		//move outwards till you intersect an object...

		//This algorithm is apparently called DDA : http://lodev.org/cgtutor/raycasting.html
//		std::cout<<"Entering the angle\n";
		//Determine ray direction
		float angle = (_theta + (i - 90.0) * M_PI / 180.0f);
		float rayDirX = cos(angle);
		float rayDirY = sin(angle);

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
			//Check if ray has hit a wall
//			std::cout<<"Check! "<<mapX<<", "<<mapY<<" \n";
			//Bounds check
			if ((mapX >= 0 && mapX < _map_ptr->get_map().cols)
					&& (mapY >= 0 && mapY < _map_ptr->get_map().rows)) {
				if (_map_ptr->get_map().at<float>(mapX, mapY) > 0)
					hit = 1;
			}
			else{
				break;
			}
		}
//		std::cout<<"DDA done\n i = "<<i<<"\n";
		//Find the distance
		double dist;
		if (hit) {
			dist = sqrt(sideDistX * sideDistX + sideDistY * sideDistY);
		} else {
			//@todo: specify this in a more smart manner
			dist = dist_max;
		}

		//Now construct fancy probability distribution here
		//@todo: learn these parameters?
		float z_hit = 1.0;
//		std::cout<<"Assigning values, hold on to sombreros\n";
		probabilities[i] = z_hit * gaussian_spread(dist, 1.0);
//		std::cout<<"Survival!\n";
		//DEBUG:: plotting these lines!!
		cv::line(image, cv::Point(_x, _y),
				cv::Point(_x + sideDistX, _y + sideDistY), CV_RGB(255,0,0));
	}
	float q = 1.0;
	for (int i = 0; i < 180; i++) {
		q *= probabilities[i];
	}
	cv::imshow("Debug", image);
	cv::waitKey(-1);
	return q;
}

float Particle::gaussian_spread(float mean_dist, float std_dev) {
	boost::mt19937 rng;
	boost::normal_distribution<> zhit_prob(mean_dist, std_dev);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(
			rng, zhit_prob);

	return var_nor();
}
