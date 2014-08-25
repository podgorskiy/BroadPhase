#pragma once
class BroadPhaseNaive : public IBroadPhase
{
public:
	BroadPhaseNaive(int bodiesCount);

	virtual void UpdateAABBList(const std::vector<IBody*>& bodiesList);
	virtual const CStyleArray<int>& GenerateOverlapList();

private:
	std::vector<aabb2df> m_aabbList;
	CStyleArray<int> m_overlapingList;
};


