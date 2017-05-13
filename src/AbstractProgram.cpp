#include "AbstractProgram.hpp"

AbstractProgram::AbstractProgram(sl::zed::ZEDResolution_mode resolution, sl::zed::MODE depthQuality, int maximumDepthDistance, const std::string& file) :
	m_camera(resolution, depthQuality, maximumDepthDistance, file),
	m_pressedKey(0),
	m_oldImageId(-1),
	m_imageHaveChanged(false)
{
}

void AbstractProgram::execute()
{
	while (ros::ok())
	{
		// Manipulation de la caméra
		m_oldImageId = m_camera.getCurrentImageId();
		m_camera.update();
		m_imageHaveChanged = m_oldImageId != m_camera.getCurrentImageId();
		computeFrame();

		// Recuperation des entrées utilisateur
		m_pressedKey = cv::waitKey(5);
		if (m_pressedKey == 'q')
			break;

		// Calcul des FPS
		m_fpsCounter.update();
		if (m_fpsCounter.isUpdated())
			std::cout << "FPS : " << (int)m_fpsCounter.getFps() << std::endl;
	}
}