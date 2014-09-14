#include "StdAfx.h"
#include "vector2d.h"
#include "aabb.h"
#include "CStyleArray.h"
#include "IBody.h"
#include "Profiler.h"
#include "IBroadPhase.h"
#include "BroadPhaseSS.h"

BroadPhaseSS::BroadPhaseSS(int bodiesCount)
{
	m_aabbList.reserve(bodiesCount);
	m_overlapingList.New(10 * bodiesCount); //rough estimate
	m_iteratorList.New(10 * bodiesCount);
	m_eventsMaxCount = 10 * bodiesCount;
	m_events.New(m_eventsMaxCount);
}

void BroadPhaseSS::UpdateAABBList(const std::vector<IBody*>& bodiesList)
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


inline bool BroadPhaseSS::Overlapping(int i0, int i1, const int* lookUp, const EndPoint* endpoints)
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

void BroadPhaseSS::UpdateIntervals(const CStyleArray<EndPoint>& endpoints, const CStyleArray<int>& lookUp)
{
	const CStyleArray<int> lookUpX(m_lookUpX);
	const CStyleArray<int> lookUpY(m_lookUpY);
	const CStyleArray<EndPoint> endPointsX(m_endPointsX);
	const CStyleArray<EndPoint> endPointsY(m_endPointsY);
	
	for (int i = 0, l = endpoints.size(); i < l; ++i)
	{
		EndPoint key = endpoints[i];
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
					if (j0 != j1 && Overlapping(j0, j1, lookUpX, endPointsX) && Overlapping(j0, j1, lookUpY, endPointsY))
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
			if (m_events.size() >= m_eventsMaxCount)
			{
				return;
			}
		}
		endpoints[j + 1] = key;
		lookUp[key.GetLookUp()] = j + 1;
	}
}

const CStyleArray<int>& BroadPhaseSS::GenerateOverlapList()
{
	TimeProfiler broadPhaseTime(this, ProfileScopes::BroadPhase);

	{
		TimeProfiler broadPhaseTime(this, ProfileScopes::UpdateEndPoint);
		UpdateEndPoints();
	}

	{
		TimeProfiler broadPhaseTime(this, ProfileScopes::UpdateIntervals);

		m_events.size() = 0;
		UpdateIntervals(m_endPointsX, m_lookUpX);
		UpdateIntervals(m_endPointsY, m_lookUpY);

		SetValue(ProfileScopes::CountOfEventsDispatched, m_events.size());
		SubmitValue(ProfileScopes::CountOfSwaps);
	}

	{
		TimeProfiler broadPhaseTime(this, ProfileScopes::ConsumeEvents);

		for (int it = 0, l = m_events.size(); it != l; ++it)
		{
			const Event e(m_events[it]);
			const int key = e.GetKey();
			if (e.GetType() == Event::Remove)
			{
				std::map<int, short>::iterator it = m_ovelapsMap.find(key);
				if (it != m_ovelapsMap.end())
				{
					unsigned short indexToDelete = it->second;
					m_overlapingList[indexToDelete] = m_overlapingList.pop_back();
					std::map<int, short>::iterator itModify = m_iteratorList[indexToDelete] = m_iteratorList.pop_back();
					itModify->second = indexToDelete;
					m_ovelapsMap.erase(it);
				}
			}
			else
			{
				std::map<int, short>::iterator it = m_ovelapsMap.find(key);
				if (it == m_ovelapsMap.end())
				{
					std::pair<std::map<int, short>::iterator, bool> inserted = m_ovelapsMap.insert(std::pair<int, short>(key, m_overlapingList.size()));
					m_overlapingList.push_back(key);
					m_iteratorList.push_back(inserted.first);
				}
			}
		}
	}

	SubmitValue(ProfileScopes::CountOfChecksInBroadPhase);
	SetValue(ProfileScopes::CountOfPotentialCollisions, m_overlapingList.size());
	return m_overlapingList;
}
