#ifndef _DETECTPEOPLESPROGRAM
#define _DETECTPEOPLESPROGRAM

#include <iostream>
#include <vector>
#include "AbstractProgram.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

class DetectPeoplesProgram : public AbstractProgram
{
public:
	DetectPeoplesProgram(const std::string& file = "");

	void computeFrame();

private:
	cv::Mat m_colorImage;
	cv::Mat m_grayImage;
	cv::Mat m_cannyImage;
	cv::Mat m_depthImage;
//         CascadeClassifier detectorBody;
//         vector<Rect> human;
	int m_detectionParameter1;
	int m_detectionParameter2;
};

#endif
