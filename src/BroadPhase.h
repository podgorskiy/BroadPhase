class BroadPhase
{
public:
	BroadPhase(int bodiesCount);

	void UpdateAABBList(std::vector<IBody*> bodiesList);
	const std::vector<std::pair<int, int> >& GenerateOverlapList();
	unsigned long long GetConumedTime();
	int GetNumOfChecks();
	int GetNumOfPotentialCollisions();

private:
	std::vector<aabb2df> m_aabbList;
	std::vector<std::pair<int, int> > m_overlapingList;

	unsigned long long m_consumedTime;
	int m_countOfChecks;
	int m_countOfPotentialCollisions;
};
