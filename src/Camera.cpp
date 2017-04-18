#include "Camera.hpp"

Camera::Camera(sl::zed::ZEDResolution_mode resolution, sl::zed::MODE depthQuality, int maximumDepthDistance,
	const std::string& readFile) :
	m_canRecord(false),
	m_isRecording(false),
	m_readFile(readFile),
	m_isPlayingStreaming(false),
	m_lastImagePosition(0)
{
	initialize(resolution, depthQuality, maximumDepthDistance, readFile);
}

Camera::~Camera()
{
	if (m_canRecord)
		m_zedCamera->stopRecording();
	if (m_zedCamera != nullptr)
		delete m_zedCamera;
}

bool Camera::canRecord() const
{
	return m_canRecord;
}

bool Camera::isRecording() const
{
	return m_isRecording;
}

bool Camera::isPlayingStreaming() const
{
	return m_isPlayingStreaming;
}

cv::Size Camera::getImageSize() const
{
	int width = m_zedCamera->getImageSize().width;
	int height = m_zedCamera->getImageSize().height;

	return cv::Size(width, height);
}

int Camera::getMaximumDepthDistance() const
{
	return m_zedCamera->getDepthClampValue() / 1000;
}

float Camera::getDistanceOfGreyLevel(int grayLevel) const
{
	return grayLevel / 255. * getMaximumDepthDistance();
}

cv::Mat Camera::getLeftColorImage() const
{
	return slMat2cvMat(m_zedCamera->retrieveImage(sl::zed::SIDE::LEFT));
}

cv::Mat Camera::getRightColorImage() const
{
	return slMat2cvMat(m_zedCamera->retrieveImage(sl::zed::SIDE::RIGHT));
}

cv::Mat Camera::getDepthImage() const
{
	return slMat2cvMat(m_zedCamera->normalizeMeasure(sl::zed::MEASURE::DEPTH));	// DEPTH, CONFIDENCE ou DISPARITY
}

std::vector<float> Camera::getCloudPoint() const
{
	float* data = (float*) m_zedCamera->retrieveMeasure(sl::zed::MEASURE::XYZRGBA).data;
	return {data, data + getImageSize().width * getImageSize().height * 4};
}

sl::zed::Mat Camera::getGPUCloudPoint() const
{
	return m_zedCamera->retrieveMeasure_gpu(sl::zed::MEASURE::XYZRGBA);
}

CUcontext Camera::getCudaContext() const
{
	return m_zedCamera->getCUDAContext();
}

void Camera::recreate(sl::zed::ZEDResolution_mode resolution, sl::zed::MODE depthQuality, int maximumDepthDistance, const std::string& file)
{
	if (m_canRecord)
		m_zedCamera->stopRecording();
	delete m_zedCamera;

	m_canRecord = false;
	m_isRecording = false;
	m_readFile = file;
	initialize(resolution, depthQuality, maximumDepthDistance, file);
}

void Camera::update()
{
	if (!m_zedCamera->grab(sl::zed::SENSING_MODE::STANDARD))
	{
		if (!m_canRecord && m_readFile != "")	// streaming
		{
			if (!m_isPlayingStreaming)
				m_zedCamera->setSVOPosition(m_lastImagePosition);
		}
		else if (m_canRecord && m_isRecording)	// enregistrement
		{
			m_zedCamera->record();
		}
	}
}

void Camera::enableRecording(const std::string& file)
{
	m_zedCamera->enableRecording(file, sl::zed::LOSSLESS_BASED);
	m_canRecord = true;
}

void Camera::startRecording()
{
	m_isRecording = true;
}

void Camera::stopRecording()
{
	m_isRecording = false;
}

void Camera::doStreamingAction(StreamingAction streamingAction)
{
	m_isPlayingStreaming = streamingAction == StreamingAction::Play;

	if (streamingAction == StreamingAction::GoToPreviousImage && m_zedCamera->getSVOPosition() > 0)
		m_lastImagePosition = m_zedCamera->getSVOPosition() - 1;
	else if (streamingAction == StreamingAction::GoToNextImage)
		m_lastImagePosition = m_zedCamera->getSVOPosition() + 1;
	else if (streamingAction == StreamingAction::Reload)
		m_lastImagePosition = 0;
	else
		m_lastImagePosition = m_zedCamera->getSVOPosition();

	if (streamingAction != StreamingAction::Play)
		m_zedCamera->setSVOPosition(m_lastImagePosition);
}

void Camera::initialize(sl::zed::ZEDResolution_mode resolution, sl::zed::MODE depthQuality, int maximumDepthDistance, const std::string& file)
{
	// Initialisation des parametres
	m_parameters.mode = sl::zed::MODE::QUALITY;        // NONE, PERFORMANCE, MEDIUM ou QUALITY
	m_parameters.unit = sl::zed::UNIT::MILLIMETER;
	m_parameters.verbose = 1;
	m_parameters.device = 0;
	m_parameters.coordinate = sl::zed::COORDINATE_SYSTEM::RIGHT_HANDED;


	// Initialisation de la camera
	if (file != "")
		m_zedCamera = new sl::zed::Camera(file);
	else
		m_zedCamera = new sl::zed::Camera(resolution);	// VGA, HD720, HD1080 ou HD2K
	sl::zed::ERRCODE err = m_zedCamera->init(m_parameters);

	if (err != sl::zed::ERRCODE::SUCCESS)
	{
		// on quitte le programmme en cas d'erreur
		std::cout << errcode2str(err) << std::endl;
		delete m_zedCamera;
		exit(1);
	}

	m_zedCamera->setDepthClampValue(maximumDepthDistance);
}
