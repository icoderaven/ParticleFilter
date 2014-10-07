#include "Map.h"
#include "LogReader.h"
#include "Particle.h"
#include "MCFilter.h"

int main(int argc, char *argv[]) {
	//Read Map
	Map my_map("/home/icoderaven/CMU/robostats/lab_1/data/map/wean.dat");
	if (!my_map.read_file()) {
		return 0;
	}
	//Display the map
	cv::Mat temp;
	cv::cvtColor(my_map.get_map(), temp, CV_GRAY2BGR);

	cv::imshow("Map", temp);

//	cv::waitKey(-1);
	//Read logs
	LogReader my_logs(
			"/home/icoderaven/CMU/robostats/lab_1/data/log/robotdata1.log");
	if (!my_logs.read_file()) {
		return 0;
	}
	my_logs._lasers[0].sensed_locations();
	Particle::determine_valid_locations(&my_map);
//	for (int i = 0; i < my_logs._lasers.size(); i++) {
//		cv::circle(temp,
//				cv::Point(-my_logs._lasers[i].getX(), -my_logs._lasers[i].getY()),
//				1, CV_RGB(255, 0, 0), 1);
//	}
//	cv::circle(temp, cv::Point(410, 300), 1, CV_RGB(255, 0,0));
	cv::imshow("Tracks", Particle::valid_locations_map);

//Test a particle
//	Particle p(398, 384, M_PI/2.0+0.15, &my_map);
//	p.evaluate_measurement_probability(my_logs._lasers[20], 1);
//	//Test propogation
//	p.markParticle(&temp);
//	for (unsigned int i = 0; i < my_logs._lasers.size() - 1; i++) {
//		cv::cvtColor(my_map.get_map(), temp, CV_GRAY2BGR);
//		p.markParticle(&temp);
////		for (int j = 0; j < 1; j++)
////			p.propogate(my_logs._lasers[i], my_logs._lasers[i+1]).markParticle(
////					&temp);
//		p = p.propogate(my_logs._lasers[i], my_logs._lasers[i+1]);
//		p.markParticle(&temp);
//		cv::imshow("Particle", temp);
//		p.evaluate_measurement_probability(my_logs._lasers[i], 1);
//		cv::waitKey(-1);
//	}
	MCFilter filter(1000, &my_map);
	filter.init();

	for (unsigned int i = 0; i < my_logs._lasers.size()-1; i++) {
		filter.loop(my_logs._lasers[i], my_logs._lasers[i+1]);
		filter.show_particles(&my_map);
	}
	return 1;
}
