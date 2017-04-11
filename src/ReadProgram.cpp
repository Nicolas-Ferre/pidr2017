#include "ReadProgram.hpp"

ReadProgram::ReadProgram(const std::string& file) :
	AbstractProgram(sl::zed::ZEDResolution_mode::VGA, sl::zed::MODE::QUALITY, 10000, file),
	m_colorImage(m_camera.getImageSize(), CV_8UC1),
	m_depthImage(m_camera.getImageSize(), CV_8UC3)
{
	std::cout << "Commands : " << std::endl;
	std::cout << "\t- p : play / pause video" << std::endl;
	std::cout << "\t- r : restart video" << std::endl;
	std::cout << "\t- l : go to previous image" << std::endl;
	std::cout << "\t- m : go to next image" << std::endl;
	std::cout << "\t- q : exit" << std::endl;
}

void ReadProgram::computeFrame()
{
	m_camera.getLeftColorImage().copyTo(m_colorImage);
	m_camera.getDepthImage().copyTo(m_depthImage);

	cv::imshow("Profondeur", m_depthImage);
	cv::imshow("Couleur", m_colorImage);

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
