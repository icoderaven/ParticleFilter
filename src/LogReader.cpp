#include "LogReader.h"
int LogReader::read_file() {
	std::ifstream in(_logName.c_str());
	if (!in) {
		std::cout << "\n[LogReader] Couldn\'t read file: " << _logName.c_str()
				<< " ! WHY??\n";
		return 0;
	}
	std::cout << "\n[LogReader] Reading log file " << _logName;

	std::string line;
	std::cout << "\n[LogReader] Running through the file...";
	while (std::getline(in, line)) {
		std::istringstream iss(line);
		char val;
		iss >> val;
		//Check whether this is Odometry or Laser data
		if (val == 'O') {
			float val[4];
			iss >> val[0] >> val[1] >> val[2] >> val[3];
			_odoms.push_back(OdometryData(val[0], val[1], val[2], val[3]));
		} else if (val == 'L') {
			float val[7];
			float ranges[180];
			iss >> val[0] >> val[1] >> val[2] >> val[3] >> val[4] >> val[5];
			//Now read in the ranges
			for (int i = 0; i < 180; i++) {
				iss >> ranges[i];
			}
			iss >> val[6];
			_lasers.push_back(
					LaserData(val[0], val[1], val[2], val[3], val[4], val[5],
							ranges, val[6]));
		} else {
			std::cout
					<< "\n[LogReader] The first character, it....is not a measurement!!!";
		}
	}
	std::cout << "\n[LogReader] Done reading log!";
	std::cout << "\nOdometry logs: " << _odoms.size() << " Laser logs: "
			<< _lasers.size() << "\n";
	return 1;
}

Eigen::Matrix3Xd LaserData::sensed_locations() {
	//Create a transform to move all these points from the reference frame of the laser to the world
	Eigen::Affine2d laser_to_world = Eigen::Affine2d::Identity();
	laser_to_world.rotate(_thetal);
	laser_to_world.pretranslate(Eigen::Vector2d(_xl, _yl));
	//	std::cout<<_xl<<", "<<_yl<<", "<<_thetal<<"\n";
	//std::cout<<laser_to_world.matrix();

	//Use the angular measurements to estimate where the obstacles are
	Eigen::Matrix3Xd p_in_laser(3, 180);
	std::cout << "\n" << p_in_laser.rows() << ", " << p_in_laser.cols() << "\n";
	std::cout << "\n" << laser_to_world.matrix().rows() << ", "
			<< laser_to_world.matrix().cols() << "\n";
	for (int i = 0; i < 180; i++) {
		p_in_laser(0, i) = _ranges[i] * cos((i + 1) * M_PI / 180.0f);
		p_in_laser(1, i) = _ranges[i] * sin((i + 1) * M_PI / 180.0f);
		p_in_laser(2, i) = 1;
	}
	//Finally, transform these points into the world frame
	// new_pts = laser_to_world * p_in_laser     3x3*3x180
	Eigen::Matrix3Xd new_pts(3, 180);
//	std::cout<<"\n\n"<<p_in_laser.transpose()<<"\n";
	new_pts = laser_to_world.matrix() * p_in_laser;
//	std::cout<<"\n\n"<<new_pts.transpose()<<"\n";
	return new_pts;
}
