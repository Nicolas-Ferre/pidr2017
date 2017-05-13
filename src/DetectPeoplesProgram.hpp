#ifndef _DETECTPEOPLESPROGRAM
#define _DETECTPEOPLESPROGRAM

#include <iostream>
#include <vector>
#include "AbstractProgram.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "People.hpp"

struct DetectPeople
{
	DetectPeople()
	{
		m_detectionParameter1 = 400;
		m_detectionParameter2 = 10;   // accumulator threshold, if small, can create noises
		peoples.reserve(100);

		bool loaded1 = detectorBody.load("haarcascade_fullbody.xml");
		hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
		peopleNumber = 0;
	}

	cv::CascadeClassifier detectorBody;
	cv::HOGDescriptor hog;
	std::vector<cv::Rect> human1;
	std::vector<cv::Rect> human2;

	int peopleNumber;
	std::vector<People> peoples;
	int m_detectionParameter1;
	int m_detectionParameter2;
};

class DetectPeoplesProgram : public AbstractProgram
{
public:
	DetectPeoplesProgram(const std::string& file = "");

	void computeFrame();

	static std::vector<People> getPeople(DetectPeople& detection, const cv::Mat& colorImage);

private:
	cv::Mat m_colorImage;
	cv::Mat m_grayImage;
	cv::Mat m_cannyImage;
	cv::Mat m_depthImage;
	DetectPeople m_detection;
};

#endif
