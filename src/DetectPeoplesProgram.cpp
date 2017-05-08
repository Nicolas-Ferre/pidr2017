#include "DetectPeoplesProgram.hpp"

DetectPeoplesProgram::DetectPeoplesProgram(const std::string& file) :
	AbstractProgram(sl::zed::ZEDResolution_mode::HD720, sl::zed::MODE::QUALITY, 10000, file),
	m_colorImage(m_camera.getImageSize(), CV_8UC1),
	m_grayImage(m_camera.getImageSize(), CV_8UC1),
	m_depthImage(m_camera.getImageSize(), CV_8UC3),
	m_detectionParameter1(400),
	m_detectionParameter2(10)   // accumulator threshold, if small, can create noises

        {
	std::cout << "Commands : " << std::endl;

	if (file != "")
	{
		std::cout << "\t- p : play / pause video" << std::endl;
		std::cout << "\t- r : restart video" << std::endl;
		std::cout << "\t- l : go to previous image" << std::endl;
		std::cout << "\t- m : go to next image" << std::endl;
	}

	std::cout << "\t- u : decrement param1" << std::endl;
	std::cout << "\t- i : increment param1" << std::endl;
	std::cout << "\t- j : decrement param2" << std::endl;
	std::cout << "\t- k : increment param2" << std::endl;
	std::cout << "\t- q : exit" << std::endl;
}

void DetectPeoplesProgram::computeFrame()
{
	if (m_pressedKey == 'u')
		--m_detectionParameter1;
	else if (m_pressedKey == 'i')
		++m_detectionParameter1;
	else if (m_pressedKey == 'j')
		--m_detectionParameter2;
	else if (m_pressedKey == 'k')
		++m_detectionParameter2;

	std::cout << "Detection parameters : " << m_detectionParameter1 << " / " << m_detectionParameter2 << std::endl;

	// Récupération des entrées utilisateur
	if (m_camera.fileIsRead())
	{
		if (m_pressedKey == 'p' && !m_camera.isPlayingStreaming()) // lancer la video
			m_camera.doStreamingAction(StreamingAction::Play);
		else if (m_pressedKey == 'p' && m_camera.isPlayingStreaming()) // arrêter la video
			m_camera.doStreamingAction(StreamingAction::Pause);
		else if (m_pressedKey == 'r') // relancer la video
			m_camera.doStreamingAction(StreamingAction::Reload);
		else if (m_pressedKey == 'l') // arrêter la video et revenir d'une frame
			m_camera.doStreamingAction(StreamingAction::GoToPreviousImage);
		else if (m_pressedKey == 'm') // arrêter la video et avancer d'une frame
			m_camera.doStreamingAction(StreamingAction::GoToNextImage);
	}

	//Récuperation des images
	m_camera.getLeftColorImage().copyTo(m_colorImage);
	m_camera.getDepthImage().copyTo(m_depthImage);
        cv::Mat smallImg;
        cv::resize(m_colorImage, smallImg, cv::Size(640, 480));
        cv::Mat sIB;
         cv::cvtColor(smallImg, sIB, CV_BGR2GRAY);
    

	// Détection des personnes
         

        bool loaded1 = detectorBody.load("haarcascade_fullbody.xml");
        hog.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
        hog.detectMultiScale(sIB, human2, 0, cv::Size(8,8), cv::Size(32, 32), 1.05, 2);
        if (human2.size() > 0) {
        for (int gg = 0; gg < human2.size(); gg++) {
        rectangle(smallImg, human2[gg].tl(), human2[gg].br(), cv::Scalar(0,255,0), 2, 8, 0);
        
            }
        }
        detectorBody.detectMultiScale(sIB, human1, 1.1, 1, 0 | 1, cv::Size(40,70), cv::Size(160, 600));
        if (human1.size() > 0) {
        for (int gg = 0; gg < human1.size(); gg++) {
        rectangle(smallImg, human1[gg].tl(), human1[gg].br(), cv::Scalar(0,0,255), 2, 8, 0);
        
            }
        }
	// Affichage
	cv::imshow("Niveaux de couleur", smallImg);
}
