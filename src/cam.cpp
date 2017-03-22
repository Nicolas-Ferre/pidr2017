#include <iostream>
#include <thread>
#include <sstream>
#include "Camera.hpp"
#include "Service.hpp"
#include "FpsCounter.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include "beginner_tutorials/CamToAlg.h"

int lineDepthFrame = 0;
std::vector<std::vector<float>> lineDepth;

bool add(beginner_tutorials::CamToAlg::Request  &req, beginner_tutorials::CamToAlg::Response &res)
{
	std::vector<float> depthLine(lineDepth[0].size(), 0);

	for (int i = 0; i < depthLine.size(); ++i)
	{
		for (int j = 0; j < 20; ++j)
		depthLine[i] += lineDepth[j][i];
		depthLine[i] /= 20;
	}

	res.tranche = depthLine;
	return true;
}

int main(int argc, char **argv)
{
	srand(time(0));

	// Initialisation de l'affichage
	Camera camera;
	int width = camera.getImageSize().width;
	int height = camera.getImageSize().height;
	cv::Size size(width, height); // taille de l'image

	cv::Mat depthImage(size, CV_8UC1); // image de profondeur


	// Boucle d'affichage
	bool firstFrame = true;
	FpsCounter fpsCounter;
	Service::init(argc, argv, "cam");
	Service service("claques", add);

	lineDepth = std::vector<std::vector<float>>(20, std::vector<float>(width, 0));

	while (ros::ok())
	{
		// Récupération des informations de la caméra
		camera.update();
		camera.getDepthImage().copyTo(depthImage);


		// Récupération de la profondeur de la ligne horizontale centrale
		for (int i = 0; i < width; ++i)
		{
			lineDepth[lineDepthFrame][i] =
				camera.getDistanceOfGreyLevel(255 - depthImage.at<uchar>(cv::Point(4 * i, height / 2)));
		}

		lineDepthFrame = (lineDepthFrame + 1) % 20;


		// Affichage des images
		for (int i = 0; i < width; ++i)
			depthImage.at<cv::Vec4b>(cv::Point(i, height / 2)) = cv::Vec4b(255, 0, 0, 255);

		cv::imshow("Profondeur", depthImage);


		// Récupération des inputs
		char key = cv::waitKey(5);
		if (key == 'q')
			break;


		// Calcul des FPS
		firstFrame = false;
		fpsCounter.update();
		if (fpsCounter.isUpdated())
			std::cout << "FPS : " << (int)fpsCounter.getFps() << std::endl;
	}

	return 0;
}

