#include "Camera.hpp"

Camera::Camera(sl::zed::ZEDResolution_mode resolution, sl::zed::MODE depthQuality, int maximumDepthDistance)
{
	initialize(resolution, depthQuality, maximumDepthDistance);
}

Camera::~Camera()
{
}

sl::zed::resolution Camera::getImageSize() const
{
	return m_zedCamera->getImageSize();
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

sl::zed::Camera* Camera::getCamera()
{
	return m_zedCamera;
}

void Camera::update()
{
	m_zedCamera->grab(sl::zed::SENSING_MODE::FILL);	// STANDARD ou FILL
}

void Camera::initialize(sl::zed::ZEDResolution_mode resolution, sl::zed::MODE depthQuality, int maximumDepthDistance)
{
	// Initialisation des parametres
        sl::zed::InitParams parameters;
        parameters.mode = sl::zed::MODE::QUALITY;        // NONE, PERFORMANCE, MEDIUM ou QUALITY
        parameters.unit = sl::zed::UNIT::MILLIMETER;
        parameters.verbose = 1;
        parameters.device = 0;
        
        
        // Initialisation de la camera
        m_zedCamera = new sl::zed::Camera(sl::zed::ZEDResolution_mode::VGA);	// VGA, HD720, HD1080 ou HD2K
        sl::zed::ERRCODE err = m_zedCamera->init(parameters);
        
        if (err != sl::zed::ERRCODE::SUCCESS)
        {
                // on quitte le programmme en cas d'erreur
                std::cout << errcode2str(err) << std::endl;
                delete m_zedCamera;
                exit(1);
        }
        
        m_zedCamera->setDepthClampValue(maximumDepthDistance);
}

