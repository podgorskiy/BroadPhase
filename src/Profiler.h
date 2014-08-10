#pragma once
#include "config.h"

class IParameter;

class PerformanceMeasurementsHelper
{
	friend class IParameter;

public:
	static int GetParameter(ProfileScopes::Scope scope);

protected:
	void SetValue(ProfileScopes::Scope scope, long value);
	void IncValue(ProfileScopes::Scope scope);
	void SubmitValue(ProfileScopes::Scope scope);

private:
	static int m_parameters[ProfileScopes::CountOfProfileScopes];
	static int m_pAccumulator[ProfileScopes::CountOfProfileScopes];
	static int m_pAccumulatorItCount[ProfileScopes::CountOfProfileScopes];
	unsigned long m_consumedTime;
};

class IParameter
{
public:
	IParameter(PerformanceMeasurementsHelper* owner, ProfileScopes::Scope scope)
		: m_value(0), m_owner(owner), m_scope(scope)
	{};

	virtual ~IParameter();

protected:
	long m_value;
	PerformanceMeasurementsHelper* m_owner;
	ProfileScopes::Scope  m_scope;
};

class TimeProfiler : public IParameter
{
public:
	TimeProfiler(PerformanceMeasurementsHelper* owner, ProfileScopes::Scope scope);

	virtual ~TimeProfiler();
	long GetTime();

private:
	unsigned long long GetTicks();
	unsigned long long GetTicksPerSecond();
	unsigned long long m_startTime;
};

inline void PerformanceMeasurementsHelper::SetValue(ProfileScopes::Scope scope, long value)
{
    m_pAccumulator[scope] += value;
    SubmitValue(scope);
}

inline void PerformanceMeasurementsHelper::IncValue(ProfileScopes::Scope scope)
{
	++m_pAccumulator[scope];
}

inline void PerformanceMeasurementsHelper::SubmitValue(ProfileScopes::Scope scope)
{
    int iteration = ++m_pAccumulatorItCount[scope];
    if (iteration >= OUTPUT_AVERAGE_OF)
    {
        m_pAccumulatorItCount[scope] = 0;
        m_parameters[scope] = m_pAccumulator[scope] / iteration;
        m_pAccumulator[scope] = 0;
    }
}

inline IParameter::~IParameter()
{
	m_owner->SetValue(m_scope, m_value);
};

void SleepMS(int ms);
