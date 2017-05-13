#ifndef _DETECTCIRLESPROGRAM
#define _DETECTCIRLESPROGRAM

#include <iostream>
#include "AbstractProgram.hpp"

class DetectCirclesProgram : public AbstractProgram
{
public:
	DetectCirclesProgram(const std::string& file = "");

	static float getCircleDepthErrorRatio(const cv::Mat& depthImage, const cv::Point& center, float radius);

	static float getCircleDepthDifference(const cv::Mat& depthImage, const cv::Point& center, float radius);

	static std::vector<cv::Vec3f> getBalls(const cv::Mat& colorImage, const cv::Mat& depthImage);

	void computeFrame();

private:
	cv::Mat m_colorImage;
	cv::Mat m_depthImage;
};

#endif
