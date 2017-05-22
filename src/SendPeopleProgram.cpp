#include "SendPeopleProgram.hpp"

std::vector<float> SendPeopleProgram::s_peoplePositions;
std::vector<float> SendPeopleProgram::s_ballsPositions;
SendPeopleProgram* SendPeopleProgram::s_sendPeopleObjectToTreat = nullptr;

SendPeopleProgram::SendPeopleProgram(const std::string& file) :
	AbstractProgram(sl::zed::ZEDResolution_mode::VGA, sl::zed::MODE::QUALITY, 10000, file),
	m_colorImage(m_camera.getImageSize(), CV_8UC1),
	m_depthImage(m_camera.getImageSize(), CV_8UC1),
	m_service("people", add)
{
	std::cout << "Commands : " << std::endl;
	std::cout << "\t- q : exit" << std::endl;
}

void SendPeopleProgram::computeFrame()
{
	if (m_imageHaveChanged)
	{
		s_sendPeopleObjectToTreat = this;
		m_camera.getLeftColorImage().copyTo(m_colorImage);
		m_camera.getDepthImage().copyTo(m_depthImage);
		cv::resize(m_colorImage, m_colorImage, m_colorImage.size() / 2);
		cv::resize(m_depthImage, m_depthImage, m_depthImage.size() / 2);


		// Récupération des personnes détectées
		std::vector<People> detectedPeople = DetectPeoplesProgram::getPeople(m_peopleDetection, m_colorImage);
		s_peoplePositions.clear();

		for (int i = 0; i < detectedPeople.size(); ++i)
		{
			if (detectedPeople[i].getTl().x != detectedPeople[i].getBr().x && detectedPeople[i].getTl().y != detectedPeople[i].getBr().y)
			{
				cv::Vec3f objectPosition = getReal2DPosition(detectedPeople[i].getPos());
				s_peoplePositions.push_back(objectPosition[0]);
				s_peoplePositions.push_back(objectPosition[1]);
				cv::circle(m_colorImage, detectedPeople[i].getPos(), 4, cv::Scalar(0, 0, 255), 2, 8, 0);
			}
		}


		// Récupération des ballons
		std::vector<cv::Vec3f> detectedBalls = DetectCirclesProgram::getBalls(m_colorImage, m_depthImage);
		s_ballsPositions.clear();

		for (int i = 0; i < detectedBalls.size(); ++i)
		{
			cv::Vec3f objectPosition = getReal2DPosition(cv::Point(detectedBalls[i][0], detectedBalls[i][1]));
			s_ballsPositions.push_back(objectPosition[0]);
			s_ballsPositions.push_back(objectPosition[1]);
			s_ballsPositions.push_back(objectPosition[2]);
			cv::circle(m_colorImage, cv::Point(detectedBalls[i][0], detectedBalls[i][1]), 4, cv::Scalar(0, 255, 0), 2, 8, 0);
		}


		// Affichage
		cv::imshow("Profondeur", m_colorImage);
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
}

bool SendPeopleProgram::add(beginner_tutorials::CamToAlgPeople::Request &req, beginner_tutorials::CamToAlgPeople::Response &res)
{
	if (s_sendPeopleObjectToTreat != nullptr)
	{
		if (!req.ball)
			res.positions = s_peoplePositions;
		else
			res.positions = s_ballsPositions;
		return true;
	}

	return false;
}

float SendPeopleProgram::getObjectDistance(cv::Point peoplePosition)
{
	return m_camera.getDistanceOfGreyLevel(255 - m_depthImage.at<cv::Vec4b>(peoplePosition)[0]);
}

cv::Vec3f SendPeopleProgram::getReal2DPosition(cv::Point positionInScreen)
{
	float fx = m_camera.m_zedCamera->getParameters()->LeftCam.fx;
	float fy = m_camera.m_zedCamera->getParameters()->LeftCam.fy;
	float cx = m_camera.m_zedCamera->getParameters()->LeftCam.cx;
	float cy = m_camera.m_zedCamera->getParameters()->LeftCam.cy;

	float depth = getObjectDistance(positionInScreen);
	return cv::Vec3f((1.0f * positionInScreen.x * m_camera.getImageSize().width / m_colorImage.size().width - cx) * depth / fx, depth, (1.0f * positionInScreen.y * m_camera.getImageSize().height / m_colorImage.size().height - cy) * depth / fy);
}
