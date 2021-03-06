#ifndef _CAMERA
#define _CAMERA

#include <zed/Camera.hpp>

/*
	Unité utilisée : le mètre
*/

enum class StreamingAction
{
	Play,
	Pause,
	GoToPreviousImage,
	GoToNextImage,
	Reload
};

class Camera
{
public :
	Camera(sl::zed::ZEDResolution_mode resolution = sl::zed::ZEDResolution_mode::VGA,
		sl::zed::MODE depthQuality = sl::zed::MODE::QUALITY, int maximumDepthDistance = 10000,
		const std::string& readFile = "");

	~Camera();

	bool fileIsRead() const;

	bool canRecord() const;

	bool isRecording() const;

	bool isPlayingStreaming() const;

	cv::Size getImageSize() const;

	int getMaximumDepthDistance() const;

	float getDistanceOfGreyLevel(int grayLevel) const;

	cv::Mat getLeftColorImage() const;

	cv::Mat getRightColorImage() const;

	cv::Mat getDepthImage() const;

	std::vector<float> getCloudPoint() const;

	sl::zed::Mat getGPUCloudPoint() const;

	CUcontext getCudaContext() const;

	int getCurrentImageId() const;

	void recreate(sl::zed::ZEDResolution_mode resolution = sl::zed::ZEDResolution_mode::VGA,
		sl::zed::MODE depthQuality = sl::zed::MODE::QUALITY, int maximumDepthDistance = 10000, const std::string& file = "");

	void update();

	void enableRecording(const std::string& file);

	void startRecording();

	void stopRecording();

	void doStreamingAction(StreamingAction streamingAction);

	sl::zed::Camera* m_zedCamera;

private:
	void initialize(sl::zed::ZEDResolution_mode resolution, sl::zed::MODE depthQuality, int maximumDepthDistance, const std::string& file);


	bool m_canRecord;
	bool m_isRecording;
	sl::zed::InitParams m_parameters;
	std::string m_readFile;
	bool m_fileIsRead;
	bool m_isPlayingStreaming;
	int m_lastImagePosition;
};

#endif
