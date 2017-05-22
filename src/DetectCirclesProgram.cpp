#include "DetectCirclesProgram.hpp"

//int DetectCirclesProgram::detectionParameter1 = 342;
//int DetectCirclesProgram::detectionParameter2 = 6;

int DetectCirclesProgram::detectionParameter1 = 311;
int DetectCirclesProgram::detectionParameter2 = 29;

DetectCirclesProgram::DetectCirclesProgram(const std::string& file) :
	AbstractProgram(sl::zed::ZEDResolution_mode::HD720, sl::zed::MODE::QUALITY, 10000, file),
	m_colorImage(m_camera.getImageSize(), CV_8UC1),
	m_depthImage(m_camera.getImageSize(), CV_8UC1)
{
	std::cout << "Commands : " << std::endl;

	if (file != "")
	{
		std::cout << "\t- p : play / pause video" << std::endl;
		std::cout << "\t- r : restart video" << std::endl;
		std::cout << "\t- l : go to previous image" << std::endl;
		std::cout << "\t- m : go to next image" << std::endl;
	}
	std::cout << "\t- q : exit" << std::endl;
}

float DetectCirclesProgram::getCircleDepthErrorRatio(const cv::Mat& depthImage, const cv::Point& center, float radius)
{
	int errorCount = 0;
	int pixelCount = 0;

	for (int x = center.x - radius; x < center.x + radius; ++x)
	{
		for (int y = center.y - radius; y < center.y + radius; ++y)
		{
			if (pow(center.x - x, 2) + pow(center.y - y, 2) < pow(radius, 2))
			{
				if ((int)depthImage.at<cv::Vec4b>(cv::Point(x, y))[0] == 0)
					++errorCount;

				++pixelCount;
			}
		}
	}

	return 1.0f * errorCount / pixelCount;
}

float DetectCirclesProgram::getCircleDepthDifference(const cv::Mat &depthImage, const cv::Point &center, float radius)
{
	uchar minDepth = 255;
	uchar maxDepth = 0;

	for (int x = center.x - radius; x < center.x + radius; ++x)
	{
		for (int y = center.y - radius; y < center.y + radius; ++y)
		{
			if (pow(center.x - x, 2) + pow(center.y - y, 2) < pow(radius*0.5, 2))
			{
				uchar depth = depthImage.at<cv::Vec4b>(cv::Point(x, y))[0];

				if (depth < minDepth && minDepth > 10)
					minDepth = depth;
				if (depth > maxDepth)
					maxDepth = depth;
			}
		}
	}

	return maxDepth - minDepth;
}

std::vector<cv::Vec3f> DetectCirclesProgram::getBalls(const cv::Mat& colorImage, const cv::Mat& depthImage)
{
	static cv::Mat cannyImage;
	static cv::Mat grayImage;

	cv::cvtColor(colorImage, grayImage, CV_BGR2GRAY);
	equalizeHist(grayImage, grayImage);
	GaussianBlur(grayImage, grayImage, cv::Size(9, 9), 2, 2);


	const int ratio = 2;
	const int lowThreshold = detectionParameter1;
	Canny(grayImage, cannyImage, detectionParameter1 / 2, detectionParameter1);
	cv::imshow("Canny", cannyImage);
	//cv::imshow("Gray", grayImage);

	std::vector<cv::Vec3f> ballsList;
	std::cout << detectionParameter1 << " / " << detectionParameter2 << std::endl;
	cv::HoughCircles(grayImage, ballsList, CV_HOUGH_GRADIENT, 1, grayImage.rows / 8, detectionParameter1, detectionParameter2, 5, grayImage.rows / 8);

	// Elimination des faux positifs
	for (int i = ballsList.size() - 1; i >=0; --i)
	{
		cv::Point center(cvRound(ballsList[i][0]), cvRound(ballsList[i][1]));
		float radius = ballsList[i][2];
		float depthDifference = getCircleDepthDifference(depthImage, center, radius);

		if (getCircleDepthErrorRatio(depthImage, center, radius) > 0.2  // si le cercle contient plus de 20% de profondeurs à 0
			 || depthDifference > 10) // si la difference de profondeur dans le cercle est trop importante
		{
			ballsList.erase(ballsList.begin() + i);
		}
	}

	return ballsList;
}

void DetectCirclesProgram::computeFrame()
{
	//if (m_imageHaveChanged)
	{
		// Récuperation des images
		m_camera.getLeftColorImage().copyTo(m_colorImage);
		m_camera.getDepthImage().copyTo(m_depthImage);
		cv::resize(m_colorImage, m_colorImage, m_colorImage.size() / 2);
		cv::resize(m_depthImage, m_depthImage, m_depthImage.size() / 2);


		// Détection des cercles
		std::vector <cv::Vec3f> circlesList = getBalls(m_colorImage, m_depthImage);

		for (int i = 0; i < circlesList.size(); i++)
		{
			cv::Point center(cvRound(circlesList[i][0]), cvRound(circlesList[i][1]));
			int radius = cvRound(circlesList[i][2]);
			cv::circle(m_colorImage, center, radius, cv::Scalar(0, 255, 0), 1, 8, 0);
			cv::circle(m_depthImage, center, radius, cv::Scalar(0, 255, 0), 1, 8, 0);

			cv::putText(m_colorImage, std::to_string((int)getCircleDepthDifference(m_depthImage, center, radius)), center + cv::Point(radius, radius), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,255,0), 1);
		}


		// Affichage
		//cv::imshow("Profondeur", m_depthImage);
		cv::imshow("Couleur", m_colorImage);
	}

	// Récupération des entrées utilisateur
	if (m_camera.fileIsRead())
	{
		if (m_pressedKey == 'p' && !m_camera.isPlayingStreaming()) // lancer la video
			m_camera.doStreamingAction(StreamingAction::Play);
		else if (m_pressedKey == 'p' && m_camera.isPlayingStreaming()) // arrêter la video
			m_camera.doStreamingAction(StreamingAction::Pause);
		else if (m_pressedKey == 'r') // relancer la video
			m_camera.doStreamingAction(StreamingAction::Reload);
		else if (m_pressedKey == 'l') // arrêter la video et revenir d'une frame
			m_camera.doStreamingAction(StreamingAction::GoToPreviousImage);
		else if (m_pressedKey == 'm') // arrêter la video et avancer d'une frame
			m_camera.doStreamingAction(StreamingAction::GoToNextImage);
	}

	if (m_pressedKey == 'u')
		--detectionParameter1;
	else if (m_pressedKey == 'i')
		++detectionParameter1;
	else if (m_pressedKey == 'j')
		--detectionParameter2;
	else if (m_pressedKey == 'k')
		++detectionParameter2;
}
