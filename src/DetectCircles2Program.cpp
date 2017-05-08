#include "DetectCircles2Program.hpp"

DetectCircles2Program::DetectCircles2Program(const std::string& file) :
	AbstractProgram(sl::zed::ZEDResolution_mode::HD720, sl::zed::MODE::QUALITY, 10000, file),
	m_colorImage(m_camera.getImageSize(), CV_8UC1),
	m_grayImage(m_camera.getImageSize(), CV_8UC1),
	m_cannyImage(m_camera.getImageSize(), CV_8UC1)
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

void DetectCircles2Program::computeFrame()
{
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


	m_camera.getLeftColorImage().copyTo(m_colorImage);
	cv::cvtColor(m_colorImage, m_grayImage, CV_BGR2GRAY);
	const int ratio = 2;
	const int lowThreshold = 100;

	// Detection contours
	medianBlur(m_grayImage, m_grayImage, 11);
	Canny(m_grayImage, m_cannyImage, lowThreshold, lowThreshold*ratio);


	// Affichage
	cv::imshow("Output", m_grayImage);
	cv::imshow("Output 2", m_cannyImage);
}


