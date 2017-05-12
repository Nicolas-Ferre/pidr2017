#include "People.hpp"


People::People(std::string namep)
{
   name=namep;
   drop=0;
   used=true;
   fiability=176;
}


void People::setPos(cv::Point pos){
    position=pos;
}

cv::Point People::getPos(){
    return position;
}

void People::setSpeed(cv::Point vit){
    speed=vit;
}

cv::Point People::getSpeed(){
    return speed;
}

void People::addFiability(int F){
    fiability+=F;
    if (fiability>255)
        fiability=255;
    if(fiability<0)
        fiability=0;
}


int People::getFiability(){
        return fiability;
}

void People::addDrop(){
    drop+=1;
}


void People::rstDrop(){
    drop=0;
}


int People::getDrop(){
    return drop;
}

void People::setUsed(bool u){
    used=u;
}

bool People::getUsed(){
    return used;
}

void People::setRect(cv::Point x,cv::Point y){
 tl=x;
 br=y; 
}
        
cv::Point People::getTl(){
    return tl;
}

cv::Point People::getBr(){
    return br;
}

cv::Point People::getSpeedTl(){
    return speedTl;
}

void People::setSpeedTl(cv::Point sp){
    speedTl=sp;
}