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
		sl::zed::MODE depthQuality = sl::zed::MODE::QUALITY, int maximumDepthDistance = 10000, const std::string& file = "");
	
	~Camera();
	
	sl::zed::resolution getImageSize() const;
	
	int getMaximumDepthDistance() const;
	
	float getDistanceOfGreyLevel(int grayLevel) const;
	
	cv::Mat getLeftColorImage() const;
	
	cv::Mat getRightColorImage() const;
	
	cv::Mat getDepthImage() const;

	void recreate(sl::zed::ZEDResolution_mode resolution = sl::zed::ZEDResolution_mode::VGA,
		sl::zed::MODE depthQuality = sl::zed::MODE::QUALITY, int maximumDepthDistance = 10000, const std::string& file = "");

	void update();

	void enableRecording(const std::string& file);

	void startRecording();

	void stopRecording();

	void resetReading();

private:
	void initialize(sl::zed::ZEDResolution_mode resolution, sl::zed::MODE depthQuality, int maximumDepthDistance, const std::string& file);

	sl::zed::Camera* m_zedCamera;
	bool m_canRecord;
	bool m_isRecording;
	sl::zed::InitParams m_parameters;
	std::string m_readFile;
};

#endif
