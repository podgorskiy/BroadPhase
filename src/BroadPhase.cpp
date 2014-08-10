#include "StdAfx.h"
#include "vector2d.h"
#include "aabb.h"
#include "IBody.h"
#include "Profiler.h"
#include "BroadPhase.h"

BroadPhase::BroadPhase(int bodiesCount) : m_consumedTime(0), m_countOfChecks(0), m_countOfPotentialCollisions(0)
{
	m_aabbList.reserve(bodiesCount);
	m_overlapingList.reserve( 5 * bodiesCount); //rough estimate
}

void BroadPhase::UpdateAABBList(std::vector<IBody*> bodiesList)
{
	m_aabbList.resize(bodiesList.size());
	int index = 0;
	for (std::vector<IBody*>::const_iterator it = bodiesList.begin(); it != bodiesList.end(); ++it, ++index)
	{
		m_aabbList[index] = (*it)->GetAABB();
	}
}

const std::vector<std::pair<int, int> >& BroadPhase::GenerateOverlapList()
{
	TimeProfiler broadPhaseTime(this, ProfileScopes::BroadPhase);

	m_overlapingList.clear();
	int indexA = 0;
	for (std::vector<aabb2df>::const_iterator ita = m_aabbList.begin(); ita != m_aabbList.end(); ++ita, ++indexA)
	{
		int indexB = indexA + 1;
		for (std::vector<aabb2df>::const_iterator itb = ita + 1; itb != m_aabbList.end(); ++itb, ++indexB)
		{
			if (Overlapping(*ita, *itb))
			{
				m_overlapingList.push_back(std::pair<int, int>(indexA, indexB));
				IncValue(ProfileScopes::CountOfPotentialCollisions);
			}
			IncValue(ProfileScopes::CountOfChecksInBroadPhase);
		}
	}

	SubmitValue(ProfileScopes::CountOfChecksInBroadPhase);
	SubmitValue(ProfileScopes::CountOfPotentialCollisions);
	return m_overlapingList;
}
