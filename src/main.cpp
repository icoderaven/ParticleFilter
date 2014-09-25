#include "Map.h"
#include "LogReader.h"

int main(int argc, char *argv[])
{
	//Read Map
	Map my_map("/home/icoderaven/CMU/robostats/lab_1/data/map/wean.dat");
	if(!my_map.read_file())
	{
		return 0;
	}
	//Display the map
	cv::imshow("Map", my_map.get_map());
	cv::waitKey(-1);
	//Read logs
	LogReader my_logs("/home/icoderaven/CMU/robostats/lab_1/data/log/robotdata1.log");
	if(!my_logs.read_file())
	{
		return 0;
	}
	my_logs._lasers[0].sensed_locations();
	return 1;
}
