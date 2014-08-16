#pragma once
class BroadPhaseNaive : public IBroadPhase
{
public:
	BroadPhaseNaive(int bodiesCount);

	virtual void UpdateAABBList(const std::vector<IBody*> bodiesList);
	virtual const std::vector<std::pair<int, int> >& GenerateOverlapList();

private:
	std::vector<aabb2df> m_aabbList;
	std::vector<std::pair<int, int> > m_overlapingList;
};


