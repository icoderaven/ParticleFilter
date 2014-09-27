#include "Map.h"
#include "LogReader.h"

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
	for (int i = 0; i < my_logs._lasers.size(); i++) {
		cv::circle(temp,
				cv::Point(-my_logs._lasers[i].getX(), -my_logs._lasers[i].getY()),
				1, CV_RGB(255, 0, 0), 1);
	}
	cv::imshow("Tracks", temp);
	cv::waitKey(-1);
	return 1;
}
