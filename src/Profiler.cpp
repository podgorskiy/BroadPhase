#include "StdAfx.h"
#include "Profiler.h"

#ifdef WIN32
#include <windows.h>
#else
#include <time.h>
#endif

int PerformanceMeasurementsHelper::m_parameters[ProfileScopes::CountOfProfileScopes] = { 0 };
int PerformanceMeasurementsHelper::m_pAccumulator[ProfileScopes::CountOfProfileScopes] = { 0 };
int PerformanceMeasurementsHelper::m_pAccumulatorItCount[ProfileScopes::CountOfProfileScopes] = { 0 };
long long PerformanceMeasurementsHelper::m_sum[ProfileScopes::CountOfProfileScopes] = { 0 };

int PerformanceMeasurementsHelper::GetParameter(ProfileScopes::Scope scope)
{
	return m_parameters[scope];
}

long long PerformanceMeasurementsHelper::GetSum(ProfileScopes::Scope scope)
{
	return m_sum[scope];
}

TimeProfiler::TimeProfiler(PerformanceMeasurementsHelper* owner, ProfileScopes::Scope scope) : IParameter(owner, scope)
{
	m_startTime = GetTicks();
}

TimeProfiler::~TimeProfiler()
{
	m_value = GetTime();
}

long TimeProfiler::GetTime()
{
	unsigned long long difference = GetTicks() - m_startTime;
	unsigned long long tickPerSecond = GetTicksPerSecond();
	unsigned long long tickInNanoseconds = 1000000000 / tickPerSecond;
	unsigned long long tickInNanosecondsRemainder = 1000000000 % tickPerSecond;
	unsigned long long differenceInNanoSeconds =
		difference * tickInNanoseconds + (difference * tickInNanosecondsRemainder) / tickPerSecond;
	return static_cast<long>(differenceInNanoSeconds);
}

#ifdef WIN32
inline unsigned long long TimeProfiler::GetTicks()
{
	LARGE_INTEGER ticks;
	QueryPerformanceCounter(&ticks);
	return ticks.QuadPart;
}

inline unsigned long long TimeProfiler::GetTicksPerSecond()
{
	LARGE_INTEGER tickPerSeconds;
	QueryPerformanceFrequency(&tickPerSeconds);
	return tickPerSeconds.QuadPart;
}
#else
inline unsigned long long TimeProfiler::GetTicks()
{
	struct timespec t;
	clock_gettime(CLOCK_REALTIME, &t);
	unsigned long long time = t.tv_sec * 1e9 + t.tv_nsec;
	return time;
}

inline unsigned long long TimeProfiler::GetTicksPerSecond()
{
	return 1e9;
}
#endif


void SleepMS(int ms)
{
#ifdef WIN32
	Sleep(ms);
#else
    struct timespec t;
    t.tv_sec = 0;
    t.tv_nsec = ms * 1000000;
    struct timespec tRemain;
	nanosleep(&t, &tRemain);
#endif
}
