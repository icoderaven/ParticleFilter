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

	cv::waitKey(-1);
	//Read logs
	LogReader my_logs(
			"/home/icoderaven/CMU/robostats/lab_1/data/log/robotdata1.log");
	if (!my_logs.read_file()) {
		return 0;
	}
	my_logs._lasers[0].sensed_locations();
//	for (int i = 0; i < my_logs._lasers.size(); i++) {
//		cv::circle(temp,
//				cv::Point(-my_logs._lasers[i].getX(), -my_logs._lasers[i].getY()),
//				1, CV_RGB(255, 0, 0), 1);
//	}
	cv::circle(temp, cv::Point(410,300), 1, CV_RGB(255, 0,0));
//	cv::imshow("Tracks", temp);
//	cv::waitKey(-1);

	//Test a particle
	Particle p(410,300,0, &my_map);
	p.evaluate_measurement_probability(my_logs._lasers[0]);

	MCFilter filter(100);
	filter.init(&my_map);
	return 1;
}
