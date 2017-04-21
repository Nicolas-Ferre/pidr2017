#ifndef _DETECTCIRLESPROGRAM
#define _DETECTCIRLESPROGRAM

#include <iostream>
#include "AbstractProgram.hpp"

class DetectCirclesProgram : public AbstractProgram
{
public:
	DetectCirclesProgram(const std::string& file = "");

	void computeFrame();

private:
	cv::Mat m_colorImage;
	cv::Mat m_grayImage;
	cv::Mat m_cannyImage;
	cv::Mat m_depthImage;
	int m_detectionParameter1;
	int m_detectionParameter2;
};

#endif
