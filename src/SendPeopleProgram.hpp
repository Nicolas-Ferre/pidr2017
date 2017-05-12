#ifndef _SENDPEOPLEPROGRAM
#define _SENDPEOPLEPROGRAM

#include <iostream>
#include "AbstractProgram.hpp"
#include "DetectPeoplesProgram.hpp"
#include "Service.hpp"
#include "People.hpp"

class SendPeopleProgram : public AbstractProgram
{
public:
	SendPeopleProgram(const std::string& file = "");

	void computeFrame();

private:
	float getObjectMinDistance(cv::Point peoplePosition);

	static bool add(beginner_tutorials::CamToAlgPeople::Request &req, beginner_tutorials::CamToAlgPeople::Response &res);

	cv::Mat m_colorImage;
	cv::Mat m_depthImage;
	Service m_service;
	DetectPeople m_peopleDetection;
	cv::Mat m_depthTemp;

	static std::vector<float> s_peoplePositions;
	static std::vector<float> s_ballsPositions;
	static SendPeopleProgram* s_sendPeopleObjectToTreat;
};

#endif
