#ifndef _ABSTRACTPROGRAM
#define _ABSTRACTPROGRAM

#include <iostream>
#include "ros/ros.h"
#include "Camera.hpp"
#include "FpsCounter.hpp"

class AbstractProgram
{
public:
	AbstractProgram(sl::zed::ZEDResolution_mode resolution = sl::zed::ZEDResolution_mode::VGA,
		sl::zed::MODE depthQuality = sl::zed::MODE::QUALITY, int maximumDepthDistance = 10000, const std::string& file = "");

	virtual void computeFrame() = 0;

	void execute();
	
protected:
	Camera m_camera;
	FpsCounter m_fpsCounter;
	char m_pressedKey;
	int m_oldImageId;
	bool m_imageHaveChanged;
};

#endif
