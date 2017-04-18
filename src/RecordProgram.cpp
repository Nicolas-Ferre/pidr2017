#include "RecordProgram.hpp"

RecordProgram::RecordProgram(const std::string& file) :
	AbstractProgram(sl::zed::ZEDResolution_mode::HD720, sl::zed::MODE::QUALITY, 10000),
	m_colorImage(m_camera.getImageSize(), CV_8UC1),
	m_depthImage(m_camera.getImageSize(), CV_8UC3)
{
	m_camera.enableRecording(file);
	std::cout << "Commands : " << std::endl;
	std::cout << "\t- r : start/resume recording" << std::endl;
	std::cout << "\t- p : pause recording" << std::endl;
	std::cout << "\t- q : exit" << std::endl;
}

void RecordProgram::computeFrame()
{
	m_camera.getLeftColorImage().copyTo(m_colorImage);
	m_camera.getDepthImage().copyTo(m_depthImage);

	cv::imshow("Profondeur", m_depthImage);
	cv::imshow("Couleur", m_colorImage);

	if (m_pressedKey == 'r' && !m_camera.isRecording()) // lancer l'enregistrement
		m_camera.startRecording();
	else if (m_pressedKey == 'p' && m_camera.isRecording()) // mettre en pause l'enregistrement
		m_camera.stopRecording();
}
