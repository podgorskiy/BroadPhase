#include "StdAfx.h"
#include "vector2d.h"
#include "aabb.h"
#include "IBody.h"
#include "Profiler.h"
#include "IBroadPhase.h"
#include "BroadPhaseSS.h"

BroadPhaseSS::BroadPhaseSS(int bodiesCount)
{
	m_aabbList.reserve(bodiesCount);
	m_overlapingList.reserve(5 * bodiesCount); //rough estimate
	m_events.reserve(bodiesCount);	//rough estimate
}

void BroadPhaseSS::UpdateAABBList(const std::vector<IBody*> bodiesList)
{
	m_aabbList.resize(bodiesList.size());
	int index = 0;
	for (std::vector<IBody*>::const_iterator it = bodiesList.begin(); it != bodiesList.end(); ++it, ++index)
	{
		m_aabbList[index] = (*it)->GetAABB();
	}
	if (m_endPointsX.size() != 2 * bodiesList.size())
	{
		m_endPointsX.resize(2 * bodiesList.size());
		m_endPointsY.resize(2 * bodiesList.size());
		int aabbIndex = 0;
		int endPointIndex = 0;
		for (std::vector<aabb2df>::const_iterator it = m_aabbList.begin(); it != m_aabbList.end(); ++it, ++aabbIndex)
		{
			m_endPointsX[endPointIndex].Set(aabbIndex, FLT_MAX, EndPoint::Minimum);
			m_endPointsY[endPointIndex].Set(aabbIndex, FLT_MAX, EndPoint::Minimum);
			++endPointIndex;
			m_endPointsX[endPointIndex].Set(aabbIndex, FLT_MAX, EndPoint::Maximum);
			m_endPointsY[endPointIndex].Set(aabbIndex, FLT_MAX, EndPoint::Maximum);
			++endPointIndex;
		}
		m_lookUpX.resize(2 * bodiesList.size());
		m_lookUpY.resize(2 * bodiesList.size());
		endPointIndex = 0;
		for (std::vector<EndPoint>::const_iterator it = m_endPointsX.begin(); it != m_endPointsX.end(); ++it, ++endPointIndex)
		{
			m_lookUpX[it->GetLookUp()] = endPointIndex;
			m_lookUpY[it->GetLookUp()] = endPointIndex;
		}
	}
}

void BroadPhaseSS::UpdateEndPoints()
{
	int id = 0;
	for (std::vector<aabb2df>::iterator it = m_aabbList.begin(); it != m_aabbList.end(); ++it, ++id)
	{
		m_endPointsX[m_lookUpX[2 * id]].m_value = it->minp.x;
		m_endPointsY[m_lookUpY[2 * id]].m_value = it->minp.y;
		m_endPointsX[m_lookUpX[2 * id + 1]].m_value = it->maxp.x;
		m_endPointsY[m_lookUpY[2 * id + 1]].m_value = it->maxp.y;
	}
}

inline bool BroadPhaseSS::Overlapping(int i0, int i1, const std::vector<int>& lookUp, const std::vector<EndPoint>& endpoints)
{
	IncValue(ProfileScopes::CountOfChecksInBroadPhase);
	float max0 = endpoints[lookUp[2 * i0 + 1]].m_value;
	float min1 = endpoints[lookUp[2 * i1]].m_value;
	if (max0 < min1)
	{
		return false;
	}
	float min0 = endpoints[lookUp[2 * i0]].m_value;
	float max1 = endpoints[lookUp[2 * i1 + 1]].m_value;
	return min0 <= max1;
}

void BroadPhaseSS::UpdateIntervals(std::vector<EndPoint>& endpoints, std::vector<int>& lookUp)
{
	int i = 0;
	for (std::vector<EndPoint>::iterator it = endpoints.begin(); it != endpoints.end(); ++it, ++i)
	{
		EndPoint key = *it;
		int j = i - 1;
		while (j >= 0 && key < endpoints[j])
		{
			//swap
			EndPoint end0 = endpoints[j];
			EndPoint end1 = endpoints[j + 1];
			if (end0.GetType() == EndPoint::Minimum)
			{
				if (end1.GetType() == EndPoint::Maximum)
				{
					int j0 = end0.GetIndex();
					int j1 = end1.GetIndex();
					if (j0 != j1)
					{
						m_events.push_back(Event(j0, j1, Event::Remove));
					}
				}
			}
			else
			{
				if (end1.GetType() == EndPoint::Minimum)
				{
					int j0 = end0.GetIndex();
					int j1 = end1.GetIndex();
					if (j0 != j1 && Overlapping(j0, j1, m_lookUpX, m_endPointsX) && Overlapping(j0, j1, m_lookUpY, m_endPointsY))
					{
						m_events.push_back(Event(j0, j1, Event::Insert));
					}
				}
			}
			endpoints[j] = end1;
			endpoints[j + 1] = end0;
			lookUp[end1.GetLookUp()] = j;
			lookUp[end0.GetLookUp()] = j + 1;
			--j;
			IncValue(ProfileScopes::CountOfSwaps);
		}
		endpoints[j + 1] = key;
		lookUp[key.GetLookUp()] = j + 1;
	}
}

const std::vector<std::pair<int, int> >& BroadPhaseSS::GenerateOverlapList()
{
	TimeProfiler broadPhaseTime(this, ProfileScopes::BroadPhase);

	{
		TimeProfiler broadPhaseTime(this, ProfileScopes::UpdateEndPoint);
		UpdateEndPoints();
	}

	{
		TimeProfiler broadPhaseTime(this, ProfileScopes::UpdateIntervals);

		m_events.clear();
		UpdateIntervals(m_endPointsX, m_lookUpX);
		UpdateIntervals(m_endPointsY, m_lookUpY);

		SetValue(ProfileScopes::CountOfEventsDispatched, m_events.size());
		SubmitValue(ProfileScopes::CountOfSwaps);
	}

	{
		TimeProfiler broadPhaseTime(this, ProfileScopes::ConsumeEvents);

		for (std::vector<Event>::iterator it = m_events.begin(); it != m_events.end(); ++it)
		{
			if (it->GetType() == Event::Remove)
			{
				m_ovelapsSet.erase(std::make_pair(it->GetIndex0(), it->GetIndex1()));
			}
			else
			{
				m_ovelapsSet.insert(std::make_pair(it->GetIndex0(), it->GetIndex1()));
			}
		}
	}

	{
		TimeProfiler broadPhaseTime(this, ProfileScopes::CreateOverlapingList);

		m_overlapingList.clear();
		for (std::set<std::pair<int, int> >::iterator it = m_ovelapsSet.begin(); it != m_ovelapsSet.end(); ++it)
		{
			m_overlapingList.push_back(*it);
		}
	}

	SubmitValue(ProfileScopes::CountOfChecksInBroadPhase);
	SetValue(ProfileScopes::CountOfPotentialCollisions, m_overlapingList.size());
	return m_overlapingList;
}
