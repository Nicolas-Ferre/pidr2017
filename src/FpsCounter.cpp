#include "FpsCounter.hpp"

FpsCounter::FpsCounter() :
	m_frameCount(0),
        m_beginTime(clock()),
        m_totalTime(0),
        m_fps(0),
        m_isUpdated(false)
{
}

float FpsCounter::getFps() const
{
	return m_fps;
}

bool FpsCounter::isUpdated() const
{
	return m_isUpdated;
}

void FpsCounter::update()
{
	++m_frameCount;
	m_totalTime += clock() - m_beginTime;

	if (m_frameCount % 100 == 0)
	{
		m_fps = 10000 * CLOCKS_PER_SEC / m_totalTime;
		m_beginTime = clock();
		m_totalTime = 0;
		m_isUpdated = true;
	}
	else
	{
		m_isUpdated = false;
	}
}
