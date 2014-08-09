#pragma once
#include <stdio.h>

#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

class TinyProfiler
{
public:
	TinyProfiler(const char* name)
	{
		m_name = name;
		m_startTime = GetNanoSeconds();
		quiet = false;
	};

	TinyProfiler()
	{
		m_name = "";
		m_startTime = GetNanoSeconds();
		quiet = true;
	};

	unsigned long long GetTime()
	{
		quiet = true;
		return GetNanoSeconds() - m_startTime;
	};

	~TinyProfiler()
	{
		if (quiet)
			return;
		unsigned long long diffrence = GetNanoSeconds() - m_startTime;
		if (diffrence < 10000)
		{
			unsigned int d = int(diffrence);
			printf("%-50s: %6uus", m_name, d);
		}
		else
		{
			if (diffrence < 10000000)
			{
				unsigned int d = int(diffrence / 1000);
				printf("%-50s: %6ums", m_name, d);
			}
			else
			{
				unsigned int d = int(diffrence / 1000000);
				printf("%-50s: %6ds", m_name, d);
			}
		}
	};
private:

	unsigned long long GetNanoSeconds()
	{
		return (unsigned long long)(GetSecondsDouble() * 1e9);
	}
	double GetSecondsDouble()
	{
		return double(GetTicks()) / double(GetTicksPerSecond());
	}
#ifdef WIN32
	unsigned long long GetTicks()
	{
		LARGE_INTEGER ticks;
		QueryPerformanceCounter(&ticks);
		return ticks.QuadPart;
	}
	unsigned long long GetTicksPerSecond()
	{
		LARGE_INTEGER tickPerSeconds;
		QueryPerformanceFrequency(&tickPerSeconds);
		return tickPerSeconds.QuadPart;
	}

#else
unsigned long GetTicks() {
	struct timeval tp;
	gettimeofday(&tp, 0);
	double time = double(tp.tv_sec) * 1000000 + tp.tv_usec;

	return static_cast<unsigned long long>(time);
}

unsigned long long GetTicksPerSecond() {
	return 1000000;
}
#endif
	const char* m_name;
	unsigned long long m_startTime;
	bool quiet;
};
