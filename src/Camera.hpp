#ifndef _CAMERA
#define _CAMERA

#include <zed/Camera.hpp>

/*
	Unité utilisée : le mètre
*/

class Camera
{
public :
	Camera(sl::zed::ZEDResolution_mode resolution = sl::zed::ZEDResolution_mode::VGA,
		sl::zed::MODE depthQuality = sl::zed::MODE::QUALITY, int maximumDepthDistance = 10000);
	
	~Camera();
	
	sl::zed::resolution getImageSize() const;
	
	int getMaximumDepthDistance() const;
	
	float getDistanceOfGreyLevel(int grayLevel) const;
	
	cv::Mat getLeftColorImage() const;
	
	cv::Mat getRightColorImage() const;
	
	cv::Mat getDepthImage() const;
	
	sl::zed::Camera* getCamera();
	
	void update();

private:
	void initialize(sl::zed::ZEDResolution_mode resolution, sl::zed::MODE depthQuality, int maximumDepthDistance);

	sl::zed::Camera* m_zedCamera;
};

#endif
