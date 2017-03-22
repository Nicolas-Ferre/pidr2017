#ifndef _FPSCOUNTER
#define _FPSCOUNTER

#include <iostream>

class FpsCounter
{
public:
	FpsCounter();
	
	float getFps() const;
	
	bool isUpdated() const;
	
	void update();
	
private:
	int m_frameCount;
        clock_t m_beginTime;
        clock_t m_totalTime;
        float m_fps;
        bool m_isUpdated;
};

#endif
