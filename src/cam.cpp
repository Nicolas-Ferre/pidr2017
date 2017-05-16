#include <iostream>
#include <zed/Camera.hpp>

#define DEPTH_CLAMP_VALUE 10000

sl::zed::Camera* getCamera()
{
	// Initialisation des parametres
	sl::zed::InitParams parameters;
	parameters.mode = sl::zed::MODE::QUALITY;	// NONE, PERFORMANCE, MEDIUM ou QUALITY
	parameters.unit = sl::zed::UNIT::MILLIMETER;
	parameters.verbose = 1;
	parameters.device = 0;

	// Initialisation de la camera
	sl::zed::Camera* zedCamera = new sl::zed::Camera(sl::zed::ZEDResolution_mode::VGA);	// VGA, HD720, HD1080 ou HD2K
	sl::zed::ERRCODE err = zedCamera->init(parameters);

	if (err != sl::zed::ERRCODE::SUCCESS)
	{
		// on quitte le programmme en cas d'erreur
		std::cout << errcode2str(err) << std::endl;
		delete zedCamera;
		exit(1);
	}

	zedCamera->setDepthClampValue(DEPTH_CLAMP_VALUE);
	return zedCamera;
}

int main(int argc, char **argv)
{
	// Initialisation de l'affichage
	sl::zed::Camera* zedCamera = getCamera();
	int width = zedCamera->getImageSize().width;
	int height = zedCamera->getImageSize().height;
	cv::Size size(width, height); // taille de l'image

	cv::Mat depthImage(size, CV_8UC1); // image de profondeur
	cv::Mat colorLeftImage(size, CV_8UC4); // image avec le mouvement en couleur
	cv::Mat colorRightImage(size, CV_8UC4); // image avec le mouvement en couleur

	// Boucle d'affichage
	bool programShouldBeClosed = false;
	int count = 0;
	clock_t beginTime = clock();
	clock_t totalTime = 0;
	float fps = 0;

	while (!programShouldBeClosed)
	{
		zedCamera->grab(sl::zed::SENSING_MODE::STANDARD);
		slMat2cvMat(zedCamera->normalizeMeasure(sl::zed::MEASURE::DEPTH)).copyTo(depthImage);
		slMat2cvMat(zedCamera->retrieveImage(sl::zed::SIDE::LEFT)).copyTo(colorLeftImage);
		slMat2cvMat(zedCamera->retrieveImage(sl::zed::SIDE::RIGHT)).copyTo(colorRightImage);

		// Récupération des inputs
		char key = cv::waitKey(5);
		if (key == 'q')
			programShouldBeClosed = true;

		// Calcul des FPS
		totalTime += clock() - beginTime;

		if (++count % 100 == 0)
		{
			std::cout << 10000 * CLOCKS_PER_SEC / totalTime  << std::endl;
			beginTime = clock();
			totalTime = 0;
		}

		cv::imshow("Couleur gauche", colorLeftImage);
		cv::imshow("Couleur droite", colorRightImage);
		cv::imshow("Profondeur", depthImage);
	}

	delete zedCamera;
	return 0;
}



/*#include <iostream>
#include "ReadProgram.hpp"
#include "RecordProgram.hpp"
#include "Send2DMapProgram.hpp"
#include "DetectCirclesProgram.hpp"
#include "DetectPeoplesProgram.hpp"
#include "SendPeopleProgram.hpp"


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
		else if (strcmp(argv[1], "send_people") == 0)
		{
			if (argc > 3)
				std::cerr << "Usage : " << argv[0] << " " << argv[1] << " [file_to_open]" << std::endl;
			else
				SendPeopleProgram(argc == 3 ? argv[2] : "").execute();
		}
		else
		{
			std::cerr << "Unknown option \"" << argv[1] << "\"" << std::endl;
		}
	}

	return 0;
}
*/