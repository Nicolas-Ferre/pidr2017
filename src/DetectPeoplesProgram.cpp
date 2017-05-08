#include "DetectPeoplesProgram.hpp"

DetectPeoplesProgram::DetectPeoplesProgram(const std::string& file) :
	AbstractProgram(sl::zed::ZEDResolution_mode::HD720, sl::zed::MODE::QUALITY, 10000, file),
	m_colorImage(m_camera.getImageSize(), CV_8UC1),
	m_grayImage(m_camera.getImageSize(), CV_8UC1),
	m_depthImage(m_camera.getImageSize(), CV_8UC3),
	m_detectionParameter1(400),
	m_detectionParameter2(10)   // accumulator threshold, if small, can create noises

        {
	std::cout << "Commands : " << std::endl;

	if (file != "")
	{
		std::cout << "\t- p : play / pause video" << std::endl;
		std::cout << "\t- r : restart video" << std::endl;
		std::cout << "\t- l : go to previous image" << std::endl;
		std::cout << "\t- m : go to next image" << std::endl;
	}

	std::cout << "\t- u : decrement param1" << std::endl;
	std::cout << "\t- i : increment param1" << std::endl;
	std::cout << "\t- j : decrement param2" << std::endl;
	std::cout << "\t- k : increment param2" << std::endl;
	std::cout << "\t- q : exit" << std::endl;
}

void DetectPeoplesProgram::computeFrame()
{
	if (m_pressedKey == 'u')
		--m_detectionParameter1;
	else if (m_pressedKey == 'i')
		++m_detectionParameter1;
	else if (m_pressedKey == 'j')
		--m_detectionParameter2;
	else if (m_pressedKey == 'k')
		++m_detectionParameter2;

	std::cout << "Detection parameters : " << m_detectionParameter1 << " / " << m_detectionParameter2 << std::endl;

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
// // // // // // // // // // // // // // // // // // // // // // // // // // 

// 	// Récuperation des images
// 	m_camera.getLeftColorImage().copyTo(m_colorImage);
// 	m_camera.getDepthImage().copyTo(m_depthImage);
// 
// 
// 	// Détection des personnes
//  ;
//         bool loaded1 = detectorBody.load("haarcascade_fullbody.xml");
//         
// 
//         detectorBody.detectMultiScale(m_grayImage, human, 1.1, 2, 0 | 1, Size(40,70), Size(80, 300));
//    if (human.size() > 0) {
//     for (int gg = 0; gg < human.size(); gg++) {
//     //rectangle(m_colorImage, human[gg].tl(), human[gg].br(), Scalar(0,0,255), 2, 8, 0);
//         
//     }
//    }
// // // // // // // // // // // // // // // // // // // // // // // // // // // 
        
	/*cv::cvtColor(m_colorImage, m_grayImage, CV_BGR2GRAY);
	GaussianBlur(m_grayImage, m_grayImage, cv::Size(9, 9), 2, 2);
	equalizeHist(m_grayImage, m_grayImage);
	Canny(m_grayImage, m_cannyImage, m_detectionParameter1 / 2, m_detectionParameter1);

	std::vector<cv::Vec3f> circlesList;
	cv::HoughCircles(m_grayImage, circlesList, CV_HOUGH_GRADIENT, 1, m_grayImage.rows / 8, m_detectionParameter1, m_detectionParameter2, 5, m_grayImage.rows / 8);

	for(int i = 0; i < circlesList.size(); i++)
	{
		cv::Point center(cvRound(circlesList[i][0]), cvRound(circlesList[i][1]));
		int radius = cvRound(circlesList[i][2]);
		int circleGrayMean = 0;
		int circleAroundGrayMean[4] = {0};
		int circleGrayPixelCount = 0;
		int circleAroundGrayPixelCount[4] = {0};

		for (int x = center.x - radius; x < center.x + radius; ++x)
		{
			for (int y = center.y - radius; y < center.y + radius; ++y)
			{
				if (pow(center.x - x, 2) + pow(center.y - y, 2) < pow(radius, 2) && pow(center.x - x, 2) + pow(center.y - y, 2) > pow(radius / 2, 2))
				{
					circleGrayMean += m_grayImage.at<uchar>(cv::Point(x, y));
					++circleGrayPixelCount;
				}
				else
				{
					int cornerId = 0;
					if (x < center.x && y < center.y)
						cornerId = 1;
					else if (x > center.x && y < center.y)
						cornerId = 2;
					else if (x > center.x && y > center.y)
						cornerId = 3;

					circleAroundGrayMean[cornerId] += m_grayImage.at<uchar>(cv::Point(x, y));
					++circleAroundGrayPixelCount[cornerId];
				}
			}
		}

		circleGrayMean /= circleGrayPixelCount;
		for (int i = 0; i < 4; ++i)
			circleAroundGrayMean[i] /= circleAroundGrayPixelCount[i];

		int smallestDifference = INT_MAX;
		for (int i = 0; i < 4; ++i)
			if (abs(circleAroundGrayMean[i] - circleGrayMean) < smallestDifference)
				smallestDifference = abs(circleAroundGrayMean[i] - circleGrayMean);

		if (smallestDifference > 20)
			cv::circle(m_grayImage, center, radius, cv::Scalar(0,0,0), 3, 8, 0);
		else
			cv::circle(m_grayImage, center, radius, cv::Scalar(255,255,255), 3, 8, 0);
		cv::putText(m_grayImage, std::to_string(smallestDifference), center + cv::Point(radius, radius), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0,0,0), 1);
	}

*/
	// Affichage
	cv::imshow("Profondeur", m_depthImage);
	cv::imshow("Niveaux de gris", m_grayImage);
	cv::imshow("Canny", m_cannyImage);
}
