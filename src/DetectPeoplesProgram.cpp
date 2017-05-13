#include "DetectPeoplesProgram.hpp"

DetectPeoplesProgram::DetectPeoplesProgram(const std::string& file) :
	AbstractProgram(sl::zed::ZEDResolution_mode::HD720, sl::zed::MODE::QUALITY, 10000, file),
	m_colorImage(m_camera.getImageSize(), CV_8UC1),
	m_grayImage(m_camera.getImageSize(), CV_8UC1),
	m_depthImage(m_camera.getImageSize(), CV_8UC3)

        
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

std::vector<People> DetectPeoplesProgram::getPeople(DetectPeople& detection, const cv::Mat& colorImage)
{
	cv::Mat sIB;
	cv::cvtColor(colorImage, sIB, CV_BGR2GRAY);

	detection.hog.detectMultiScale(sIB, detection.human2, 0, cv::Size(8,8), cv::Size(32, 32), 1.05, 2);
	detection.detectorBody.detectMultiScale(sIB, detection.human1, 1.1, 1, 0 | 1, cv::Size(40,70), cv::Size(160, 600));


	for (int gg = 0;gg< detection.human2.size();gg++){
		cv::Point tmp=cv::Point((detection.human2[gg].tl()+detection.human2[gg].br())/2);
		bool exist=false;
		for (int hh=0;hh<detection.peoples.size();hh++){
			if (!detection.peoples[hh].getUsed() && ( cv::norm(cv::Mat(tmp),cv::Mat(detection.peoples[hh].getPos()))<(15*((255-detection.peoples[hh].getFiability())/15)+50))){
				std::cout << "nb normal "<< detection.peopleNumber << std::endl;
				exist=true;
				cv::Point oldSpeed=detection.peoples[hh].getSpeed();
				detection.peoples[hh].setSpeed(tmp-detection.peoples[hh].getPos());
				if (cv::norm(detection.peoples[hh].getSpeed())>17)
					detection.peoples[hh].setSpeed(oldSpeed);
				detection.peoples[hh].setPos(tmp);
				detection.peoples[hh].rstDrop();
				detection.peoples[hh].setUsed(true);
				detection.peoples[hh].addFiability(25);
				detection.peoples[hh].setRect(detection.human2[gg].tl(),detection.human2[gg].br());
				break;
			}
		}
		bool contains=false;
		for (int hh=0;hh<detection.peoples.size();hh++){
			if (cv::Rect(detection.peoples[hh].getTl(),detection.peoples[hh].getBr()).contains(tmp)){
				contains=true;
				break;
			}

		}
		if (!exist && !contains){
			std::cout << "nb people "<< detection.peopleNumber << std::endl;
			People np=People("people number " + std::to_string(detection.peopleNumber));
			cv::Point npos=cv::Point(tmp.x,tmp.y);
			cv::Point nspd=cv::Point(0,0);
			np.setSpeed(nspd);
			np.setPos(npos);
			np.setRect(detection.human2[gg].tl(),detection.human2[gg].br());

			detection.peoples.push_back(np);
			detection.peopleNumber+=1;
		}
	}

	for (int gg = 0;gg< detection.human1.size();gg++){
		cv::Point tmp=cv::Point((detection.human1[gg].tl()+detection.human1[gg].br())/2);
		for (int hh=0;hh<detection.peoples.size();hh++){
			if (detection.human1[gg].contains(detection.peoples[hh].getPos())){
				detection.peoples[hh].addFiability(20);
				detection.peoples[hh].setRect(detection.peoples[hh].getTl()-detection.peoples[hh].getPos()+tmp,detection.peoples[hh].getBr()-detection.peoples[hh].getPos()+tmp);
				detection.peoples[hh].setPos(tmp);
			}
		}
	}

	//mise a jour des drops pour kick
	for (int gg=0;gg< detection.peoples.size();gg++)
	{
		if (!detection.peoples[gg].getUsed())
			if (detection.peoples[gg].getFiability() > 0)
			{
				detection.peoples[gg].addDrop();
				detection.peoples[gg].addFiability(-16);
				detection.peoples[gg].setPos(detection.peoples[gg].getPos() + detection.peoples[gg].getSpeed());
			}
			else
				detection.peoples.erase(detection.peoples.begin() + gg);
	}

	for (int gg=0;gg< detection.peoples.size();gg++)
		detection.peoples[gg].setUsed(false);

	return detection.peoples;
}

void DetectPeoplesProgram::computeFrame()
{
	if (m_pressedKey == 'u')
		--m_detection.m_detectionParameter1;
	else if (m_pressedKey == 'i')
		++m_detection.m_detectionParameter1;
	else if (m_pressedKey == 'j')
		--m_detection.m_detectionParameter2;
	else if (m_pressedKey == 'k')
		++m_detection.m_detectionParameter2;

// 	std::cout << "Detection parameters : " << m_detection.m_detectionParameter1 << " / " << m_detection.m_detectionParameter2 << std::endl;

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
         

        
        m_detection.hog.detectMultiScale(sIB, m_detection.human2, 0, cv::Size(8,8), cv::Size(32, 32), 1.05, 2);
        m_detection.detectorBody.detectMultiScale(sIB, m_detection.human1, 1.1, 1, 0 | 1, cv::Size(40,70), cv::Size(160, 600));
        
//         if (m_detection.human2.size() > 0) {
//         for (int gg = 0; gg < m_detection.human2.size(); gg++) {
//         rectangle(smallImg, m_detection.human2[gg].tl(), m_detection.human2[gg].br(), cv::Scalar(0,255,0), 2, 8, 0);
//
//             }
//         }
//         if (m_detection.human1.size() > 0) {
//         for (int gg = 0; gg < m_detection.human1.size(); gg++) {
//         rectangle(smallImg, m_detection.human1[gg].tl(), m_detection.human1[gg].br(), cv::Scalar(0,0,255), 2, 8, 0);
//         
//             }
//         }
// //         
        for (int gg = 0;gg< m_detection.human2.size();gg++){
            cv::Point tmp=cv::Point((m_detection.human2[gg].tl()+m_detection.human2[gg].br())/2);
            bool exist=false;
            for (int hh=0;hh<m_detection.peoples.size();hh++){
                if (!m_detection.peoples[hh].getUsed() && ( cv::norm(cv::Mat(tmp),cv::Mat(m_detection.peoples[hh].getPos()))<(15*((255-m_detection.peoples[hh].getFiability())/15)+50))){
            std::cout << "nb normal "<< m_detection.peopleNumber << std::endl;
                    exist=true;
                    cv::Point oldSpeed=m_detection.peoples[hh].getSpeed();
                    m_detection.peoples[hh].setSpeed(tmp-m_detection.peoples[hh].getPos());
                    if (cv::norm(m_detection.peoples[hh].getSpeed())>17)
                        m_detection.peoples[hh].setSpeed(oldSpeed);
                    m_detection.peoples[hh].setPos(tmp);
                    m_detection.peoples[hh].rstDrop();
                    m_detection.peoples[hh].setUsed(true);
                    m_detection.peoples[hh].addFiability(25);
                    m_detection.peoples[hh].setRect(m_detection.human2[gg].tl(),m_detection.human2[gg].br());
                    break;
                }
            }
            bool contains=false;
            for (int hh=0;hh<m_detection.peoples.size();hh++){
            if (cv::Rect(m_detection.peoples[hh].getTl(),m_detection.peoples[hh].getBr()).contains(tmp)){
                contains=true;
                break;
            }
            
            }
            if (!exist && !contains){
                std::cout << "nb people "<< m_detection.peopleNumber << std::endl;
                People np=People("people number "/*+ std::to_string(m_detection.peopleNumber)*/);
                cv::Point npos=cv::Point(tmp.x,tmp.y);
                cv::Point nspd=cv::Point(0,0);
                np.setSpeed(nspd);
                np.setPos(npos);
                np.setRect(m_detection.human2[gg].tl(),m_detection.human2[gg].br());
                
                m_detection.peoples.push_back(np);
                m_detection.peopleNumber+=1;
            }
        }
        
        for (int gg = 0;gg< m_detection.human1.size();gg++){
                cv::Point tmp=cv::Point((m_detection.human1[gg].tl()+m_detection.human1[gg].br())/2);
                for (int hh=0;hh<m_detection.peoples.size();hh++){
                        if (m_detection.human1[gg].contains(m_detection.peoples[hh].getPos())){
                            m_detection.peoples[hh].addFiability(20);
                            m_detection.peoples[hh].setRect(m_detection.peoples[hh].getTl()-m_detection.peoples[hh].getPos()+tmp,m_detection.peoples[hh].getBr()-m_detection.peoples[hh].getPos()+tmp);
                            m_detection.peoples[hh].setPos(tmp);
                        }
                }
            }
        
            //mise a jour des drops pour kick
        for (int gg=0;gg< m_detection.peoples.size();gg++){
            if (!m_detection.peoples[gg].getUsed())
                if (m_detection.peoples[gg].getFiability()>0){
                      m_detection.peoples[gg].addDrop();  
                      m_detection.peoples[gg].addFiability(-16);
                      m_detection.peoples[gg].setPos(m_detection.peoples[gg].getPos()+m_detection.peoples[gg].getSpeed());
                }
                else 
                    m_detection.peoples.erase(m_detection.peoples.begin()+gg);
        }

        
        
//         for (int gg = 0;gg< m_detection.peoples.size();gg++){
//             circle(smallImg,m_detection.peoples[gg].getPos(),4,cv::Scalar(0,0,255), 2, 8, 0);
//             
//         }
        if (m_detection.peoples.size() > 0) {
        for (int gg = 0; gg < m_detection.peoples.size(); gg++) {
        rectangle(smallImg, m_detection.peoples[gg].getTl(), m_detection.peoples[gg].getBr(), cv::Scalar(0,m_detection.peoples[gg].getFiability(),255-m_detection.peoples[gg].getFiability()), 2, 8, 0);
        
            }
        }
        
        //fin used
        for (int gg=0;gg< m_detection.peoples.size();gg++)
            m_detection.peoples[gg].setUsed(false);
                
                
	// Affichage
	cv::imshow("Niveaux de couleur", smallImg);
}
