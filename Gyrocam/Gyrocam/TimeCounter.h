#pragma once
#include <windows.h>
#include <iostream>

class TimeCounter
{
	double PCFreq;
	__int64 CounterStart;

public:

	TimeCounter(void)
	{
		PCFreq = 0.0;
		CounterStart = 0;
	}

	void StartCounter()
	{
		LARGE_INTEGER li;
		if(!QueryPerformanceFrequency(&li))
			std::cout << "QueryPerformanceFrequency failed!\n";

		PCFreq = double(li.QuadPart)/1000.0;

		QueryPerformanceCounter(&li);
		CounterStart = li.QuadPart;
	}

	double GetCounter()
	{
		LARGE_INTEGER li;
		QueryPerformanceCounter(&li);
		return double(li.QuadPart-CounterStart)/PCFreq;
	}

	~TimeCounter(void)
	{
	}
};

