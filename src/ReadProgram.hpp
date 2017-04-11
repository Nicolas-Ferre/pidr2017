#ifndef _READPROGRAM
#define _READPROGRAM

#include <iostream>
#include "AbstractProgram.hpp"

class ReadProgram : public AbstractProgram
{
public:
	ReadProgram(const std::string& file);

	void computeFrame();

private:
	cv::Mat m_colorImage;
	cv::Mat m_depthImage;
};

#endif
