#ifndef _RECORDPROGRAM
#define _RECORDPROGRAM

#include <iostream>
#include "AbstractProgram.hpp"

class RecordProgram : public AbstractProgram
{
public:
	RecordProgram(const std::string& file);

	void computeFrame();

private:
	cv::Mat m_colorImage;
	cv::Mat m_depthImage;
};

#endif
