#include "Send2DMapProgram.hpp"

Send2DMapProgram* Send2DMapProgram::s_send2DMapObjectToTreat = nullptr;

Send2DMapProgram::Send2DMapProgram(const std::string& file) :
	AbstractProgram(sl::zed::ZEDResolution_mode::VGA, sl::zed::MODE::QUALITY, 10000, file),
	m_colorImage(m_camera.getImageSize(), CV_8UC1),
	m_depthImage(m_camera.getImageSize(), CV_8UC3),
	m_depthLineFrame(0),
	m_depthLine(20, std::vector<float>(m_camera.getImageSize().width, 0)),
	m_service("claques", add)
{
	std::cout << "Commands : " << std::endl;
	std::cout << "\t- q : exit" << std::endl;
}

void Send2DMapProgram::computeFrame()
{
	s_send2DMapObjectToTreat = this;
	m_camera.getLeftColorImage().copyTo(m_colorImage);
	m_camera.getDepthImage().copyTo(m_depthImage);


	// Récupération de la profondeur de la ligne horizontale centrale
	for (int i = 0; i < m_camera.getImageSize().width; ++i)
	{
		m_depthLine[m_depthLineFrame][i] = m_camera.getDistanceOfGreyLevel(255 -
			m_depthImage.at<uchar>(cv::Point(4 * i, m_camera.getImageSize().height / 2)));
	}

	m_depthLineFrame = (m_depthLineFrame + 1) % 20;


	// Surlignage de la ligne récupérée sur l'image de profondeur
	for (int i = 0; i < m_camera.getImageSize().width; ++i)
		m_depthImage.at<cv::Vec4b>(cv::Point(i, m_camera.getImageSize().height / 2)) = cv::Vec4b(255, 0, 0, 255);


	// Afichage
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

bool Send2DMapProgram::add(beginner_tutorials::CamToAlg::Request &req, beginner_tutorials::CamToAlg::Response &res)
{
	if (s_send2DMapObjectToTreat != nullptr)
	{
		res.fx = s_send2DMapObjectToTreat->m_camera.m_zedCamera->getParameters()->LeftCam.fx;
		res.fy = s_send2DMapObjectToTreat->m_camera.m_zedCamera->getParameters()->LeftCam.fy;
		res.cx = s_send2DMapObjectToTreat->m_camera.m_zedCamera->getParameters()->LeftCam.cx;
		res.cy = s_send2DMapObjectToTreat->m_camera.m_zedCamera->getParameters()->LeftCam.cy;
		res.posy = s_send2DMapObjectToTreat->m_camera.getImageSize().height / 2;

		std::vector<float> depthLineMean(s_send2DMapObjectToTreat->m_depthLine[0].size(), 0);

		for (int i = 0; i < depthLineMean.size(); ++i)
		{
			for (int j = 0; j < 20; ++j)
				depthLineMean[i] += s_send2DMapObjectToTreat->m_depthLine[j][i];
			depthLineMean[i] /= 20;
		}

		res.tranche = depthLineMean;
		return true;
	}

	return false;
}
