#ifndef TIMER_H
#define TIMER_H

#define WIN32_MEAN_AND_LEAN
#include <windows.h>


class Timer
{
public:
	Timer();							// OBS: S�tter precisionen p� Sleep() till 1ms (f�r testning)
	~Timer();							// S�tter tillbaka precisionen efter anv�ndning
	void start();						// Starta klockan
	void stamp();						// Ta en varvtid

	unsigned int getTimePassed() const;	// M�ter tiden mellan start() och senaste stamp() 
										// OBS: start() och stamp() _m�ste_ ha kallats
										// returnerar mikrosekunder
private:
	int calculateElapsedTime();

	LARGE_INTEGER m_timer_freq;
	LARGE_INTEGER m_counter_at_start;
	unsigned int m_last_time;
	unsigned int m_new_time;

};


#endif