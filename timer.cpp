#include "timer.h"
#include <iostream>

#pragma comment(lib, "Winmm.lib")

Timer::Timer():
	m_last_time(0), m_new_time(0)
{
	QueryPerformanceFrequency(&m_timer_freq);  
	QueryPerformanceCounter(&m_counter_at_start);  
	
	std::cout << "timerFreq_ = " << m_timer_freq.QuadPart << std::endl;  
	std::cout << "counterAtStart_ = " << m_counter_at_start.QuadPart << std::endl;  
	
	TIMECAPS ptc;  
	UINT cbtc = 8;  
	MMRESULT result = timeGetDevCaps(&ptc, cbtc);  
	if (result == TIMERR_NOERROR)  
	{    
		std::cout << "Minimum resolution = " << ptc.wPeriodMin << std::endl;
		std::cout << "Maximum resolution = " << ptc.wPeriodMax << std::endl;  
	}  
	else 
	{   
		std::cout << "result = TIMER ERROR" << std::endl;
	}

	timeBeginPeriod(1);
}

Timer::~Timer()
{
	timeEndPeriod(1);
}

int Timer::calculateElapsedTime()
{
	if (m_timer_freq.QuadPart == 0)  
	{    
		return -1;  
	}  
	else  
	{    
		LARGE_INTEGER c;    
		QueryPerformanceCounter(&c);    
		return static_cast<unsigned int>( (c.QuadPart - m_counter_at_start.QuadPart) * 1000000 / m_timer_freq.QuadPart );
	}
}

void Timer::start()
{
	m_new_time = calculateElapsedTime();
}

void Timer::stamp()
{
	m_last_time = calculateElapsedTime();
}

unsigned int Timer::getTimePassed() const
{
	return m_last_time - m_new_time;
}