#include <iostream>
#include "ReadProgram.hpp"
#include "RecordProgram.hpp"
#include "Send2DMapProgram.hpp"
#include "DetectCirclesProgram.hpp"
#include "DetectPeoplesProgram.hpp"


int main(int argc, char **argv)
{
	srand(time(0));
	Service::init(argc, argv, "cam");

	if (argc < 2)
	{
		std::cerr << "Usage : " << argv[0] << " option [parameters]" << std::endl;
		std::cerr << "Option can be : " << std::endl;
		std::cerr << "\t- read : open an existing .svo file (parameters : file_to_open)" << std::endl;
		std::cerr << "\t- record : record in a .svo file (parameters : record_file)" << std::endl;
		std::cerr << "\t- send2dmap : send depth line from the camera (or file if specified) to a 2D reconstruction ROS node (parameters : [file_to_open])" << std::endl;
		std::cerr << "\t- detect_circles : detect circles of a video or directy from the camera (parameters : [file_to_open])" << std::endl;
                std::cerr << "\t- detect_peoples : detect peoples of a video or directy from the camera (parameters : [file_to_open])" << std::endl;
	}
	else
	{
		if (strcmp(argv[1], "read") == 0)
		{
			if (argc != 3)
				std::cerr << "Usage : " << argv[0] << " " << argv[1] << " file_to_open" << std::endl;
			else
				ReadProgram(argv[2]).execute();
		}
		else if (strcmp(argv[1], "record") == 0)
		{
			if (argc != 3)
				std::cerr << "Usage : " << argv[0] << " " << argv[1] << " record_file" << std::endl;
			else
				RecordProgram(argv[2]).execute();
		}
		else if (strcmp(argv[1], "send2dmap") == 0)
		{
			if (argc > 3)
				std::cerr << "Usage : " << argv[0] << " " << argv[1] << " [file_to_open]" << std::endl;
			else
				Send2DMapProgram(argc == 3 ? argv[2] : "").execute();
		}
		else if (strcmp(argv[1], "detect_circles") == 0)
		{
			if (argc > 3)
				std::cerr << "Usage : " << argv[0] << " " << argv[1] << " [file_to_open]" << std::endl;
			else
				DetectCirclesProgram(argc == 3 ? argv[2] : "").execute();
		}
		else if (strcmp(argv[1], "detect_peoples") == 0)
		{
			if (argc > 3)
				std::cerr << "Usage : " << argv[0] << " " << argv[1] << " [file_to_open]" << std::endl;
			else
				DetectPeoplesProgram(argc == 3 ? argv[2] : "").execute();
		}
		else
		{
			std::cerr << "Unknown option \"" << argv[1] << "\"" << std::endl;
		}
	}

	return 0;
}
