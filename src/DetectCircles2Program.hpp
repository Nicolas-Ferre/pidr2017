#ifndef _DETECTCIRLES2PROGRAM
#define _DETECTCIRLES2PROGRAM

#include <iostream>
#include "AbstractProgram.hpp"

class DetectCircles2Program : public AbstractProgram
{
public:
	DetectCircles2Program(const std::string& file = "");

	void computeFrame();

private:
	cv::Mat m_colorImage;
	cv::Mat m_grayImage;
	cv::Mat m_cannyImage;
};

#endif
