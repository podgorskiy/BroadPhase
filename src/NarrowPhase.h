class NarrowPhase
{
public:
	NarrowPhase(int bodiesCount);

	const std::vector<Collision>& DetectCollisions(const std::vector<std::pair<int, int> >& overlapingList, const std::vector<IBody*>& bodies);

	unsigned long long GetConumedTime();
	int GetNumOfCollisions();
private:
	bool BallToBallCollisionTest(const Ball& ballA, const Ball& ballB, Collision& collision);
	bool BallToChainCollisionTest(const Ball& ballA, const Chain& ballB, Collision& collision);

	std::vector<Collision> collisionsList;

	unsigned long long m_consumedTime;
	int	m_collisionNumber;
};