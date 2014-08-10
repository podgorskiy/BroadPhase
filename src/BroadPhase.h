class BroadPhase : PerformanceMeasurementsHelper
{
public:
	BroadPhase(int bodiesCount);

	void UpdateAABBList(std::vector<IBody*> bodiesList);
	const std::vector<std::pair<int, int> >& GenerateOverlapList();

private:
	std::vector<aabb2df> m_aabbList;
	std::vector<std::pair<int, int> > m_overlapingList;

	unsigned long long m_consumedTime;
	int m_countOfChecks;
	int m_countOfPotentialCollisions;
};
