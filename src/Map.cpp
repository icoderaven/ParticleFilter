#include "Map.h"
boost::mt19937 Map::_gen;
int Map::read_file() {
	std::ifstream in(_mapName.c_str());
	if( !in  )
	{
		std::cout<<"\n[MapReader] Couldn\'t read file: "<<_mapName.c_str()<<" ! WHY??\n";
		return 0;
	}

	std::cout <<"\n[MapReader] Reading map " << _mapName;

	//Begin reading the headers
	std::string line;
	std::cout<<"\n[MapReader] Running through the headers...";
	while(std::getline(in, line))
	{
		std::istringstream iss(line);
		std::size_t found = line.find("global_map[0]:");
		if (found!=std::string::npos){
			std::string temp;
			iss>>temp>>_width>>_height;
			init(_width, _height);
			break;
		}
	}
	std::cout<<"\n[MapReader] Now reading map...";
	std::cout<<_map.rows<<","<<_map.cols;
	//And now read the data
	unsigned int i=0;
	while(std::getline(in, line))
	{
		std::istringstream iss(line);
		for(unsigned int j=0;j<_width;j++)
		{
			float val;
			iss>>val;
			if(val <0.0){
				_map.at<float>(i,j) = -1.0;
			}
			else{
				_map.at<float>(i,j) = 1 - val;

				if(j < _min_x){
					_min_x = j;
				}
				else if(j>_max_x){
					_max_x = j;
				}
				if(i<_min_y){
					_min_y = i;
				}
				else if(i>_max_y){
					_max_y = i;
				}
			}
		}
		i++;
	}

	cv::flip(_map, _map, 1);
	_map = _map.t();
	std::cout<<"\n[MapReader] Done reading map!";
	return 1;
}
