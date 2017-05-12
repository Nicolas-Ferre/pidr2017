#include "SendPeopleProgram.hpp"

std::vector<float> SendPeopleProgram::s_peoplePositions;
std::vector<float> SendPeopleProgram::s_ballsPositions;
SendPeopleProgram* SendPeopleProgram::s_sendPeopleObjectToTreat = nullptr;

SendPeopleProgram::SendPeopleProgram(const std::string& file) :
	AbstractProgram(sl::zed::ZEDResolution_mode::VGA, sl::zed::MODE::QUALITY, 10000, file),
	m_colorImage(m_camera.getImageSize(), CV_8UC1),
	m_depthImage(m_camera.getImageSize(), CV_8UC3),
	m_service("people", add),
	m_depthTemp(cv::Size(640, 480), CV_8UC1)
{
	std::cout << "Commands : " << std::endl;
	std::cout << "\t- q : exit" << std::endl;
}

void SendPeopleProgram::computeFrame()
{
	s_sendPeopleObjectToTreat = this;
	m_camera.getLeftColorImage().copyTo(m_colorImage);
	m_camera.getDepthImage().copyTo(m_depthImage);


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


	// Récupération des personnes détectées


	const float angleOffset = 70 / 180 * M_PI;
	std::vector<People> detectedPeople = DetectPeoplesProgram::getPeople(m_peopleDetection, m_colorImage, m_depthImage);
	s_peoplePositions.clear();

	cv::resize(m_depthImage, m_depthTemp, cv::Size(640, 480));
	for (int i = 0; i < detectedPeople.size(); ++i)
	{
		float peopleDistance = getObjectMinDistance(detectedPeople[i].getPos());
		float xImagePosition = (detectedPeople[i].getTl().x + detectedPeople[i].getBr().x) / 2;
		float angle = angleOffset + M_PI * xImagePosition * 110 / (detectedPeople.size() * 180);

		s_peoplePositions.push_back(-peopleDistance * cos(angle));
		s_peoplePositions.push_back(-peopleDistance * sin(angle));


		std::cout << peopleDistance << std::endl;

		cv::circle(m_depthTemp,detectedPeople[i].getPos(),4,cv::Scalar(0,0,255), 2, 8, 0);
	}

	// Affichage
	cv::imshow("Profondeur", m_depthTemp);
	//cv::imshow("Couleur", m_colorImage);
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

float SendPeopleProgram::getObjectMinDistance(cv::Point peoplePosition)
{
	/*if (topLeftPoint.x != bottomRightPoint.x && topLeftPoint.y == bottomRightPoint.y)
	{
		double min, max;
		cv::minMaxLoc(m_depthImage(cv::Rect(bottomRightPoint, topLeftPoint)), &min, &max);

		return m_camera.getDistanceOfGreyLevel(255 - max);
	}

	return 0;*/

	return m_depthTemp.at<uchar>(peoplePosition);
}