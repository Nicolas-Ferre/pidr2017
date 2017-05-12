#ifndef _PEOPLE
#define _PEOPLE


#include <iostream>
#include <vector>
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"



class People
{
public:
	People(std::string);
        
        void setPos(cv::Point);
        
        cv::Point getPos();

	void setSpeed(cv::Point);
        
        cv::Point getSpeed();
        
        int getFiability();
        
        void addFiability(int);
        
        void addDrop();
       
        void rstDrop();
        
        int getDrop();
        
        void setUsed(bool);
        
        bool getUsed();
        
        void setRect(cv::Point,cv::Point);
        
        cv::Point getTl();
        
        cv::Point getBr();
        
        cv::Point getSpeedTl();
        
        void setSpeedTl(cv::Point sp);
        

private:
        
        cv::Point position;
        cv::Point speed;
        cv::Point tl;
        cv::Point br;
        cv::Point speedTl;
        std::string name;
        int fiability;
        int drop;
        bool used;
};

#endif