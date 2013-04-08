#ifndef TIMER_H
#define TIMER_H

#define WIN32_MEAN_AND_LEAN
#include <windows.h>


class Timer
{
public:
	Timer();							// OBS: Sätter precisionen på Sleep() till 1ms (för testning)
	~Timer();							// Sätter tillbaka precisionen efter användning
	void start();						// Starta klockan
	void stamp();						// Ta en varvtid

	unsigned int getTimePassed() const;	// Mäter tiden mellan start() och senaste stamp() 
										// OBS: start() och stamp() _måste_ ha kallats
										// returnerar mikrosekunder
private:
	int calculateElapsedTime();

	LARGE_INTEGER m_timer_freq;
	LARGE_INTEGER m_counter_at_start;
	unsigned int m_last_time;
	unsigned int m_new_time;

};


#endif