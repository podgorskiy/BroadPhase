#include "StdAfx.h"
#include "vector2d.h"
#include "aabb.h"
#include "CStyleArray.h"
#include "IBody.h"
#include "Profiler.h"
#include "IBroadPhase.h"
#include "BroadPhaseNaive.h"

BroadPhaseNaive::BroadPhaseNaive(int bodiesCount)
{
	m_aabbList.reserve(bodiesCount);
	m_overlapingList.New(5 * bodiesCount); //rough estimate
}

void BroadPhaseNaive::UpdateAABBList(const std::vector<IBody*>& bodiesList)
{
	m_aabbList.resize(bodiesList.size());
	int index = 0;
	for (std::vector<IBody*>::const_iterator it = bodiesList.begin(); it != bodiesList.end(); ++it, ++index)
	{
		m_aabbList[index] = (*it)->GetAABB();
	}
}

const CStyleArray<int>& BroadPhaseNaive::GenerateOverlapList()
{
	TimeProfiler broadPhaseTime(this, ProfileScopes::BroadPhase);

	m_overlapingList.size() = 0;
	int indexA = 0;
	for (std::vector<aabb2df>::const_iterator ita = m_aabbList.begin(); ita != m_aabbList.end(); ++ita, ++indexA)
	{
		int indexB = indexA + 1;
		for (std::vector<aabb2df>::const_iterator itb = ita + 1; itb != m_aabbList.end(); ++itb, ++indexB)
		{
			if (Overlapping(*ita, *itb))
			{
				m_overlapingList.push_back(indexA | (indexB<<16));
			}
			IncValue(ProfileScopes::CountOfChecksInBroadPhase);
		}
	}

	SubmitValue(ProfileScopes::CountOfChecksInBroadPhase);
	SetValue(ProfileScopes::CountOfPotentialCollisions, m_overlapingList.size());
	return m_overlapingList;
}
