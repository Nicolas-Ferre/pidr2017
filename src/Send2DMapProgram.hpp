#ifndef _SEND2DMAPPROGRAM
#define _SEND2DMAPPROGRAM

#include <iostream>
#include "AbstractProgram.hpp"
#include "Service.hpp"

class Send2DMapProgram : public AbstractProgram
{
public:
	Send2DMapProgram(const std::string& file = "");

	void computeFrame();

private:
	static bool add(beginner_tutorials::CamToAlg::Request &req, beginner_tutorials::CamToAlg::Response &res);

	cv::Mat m_colorImage;
	cv::Mat m_depthImage;
	Service m_service;
	int m_depthLineFrame;
	std::vector<std::vector<float>> m_depthLine;

	static Send2DMapProgram* s_send2DMapObjectToTreat;
};

#endif
