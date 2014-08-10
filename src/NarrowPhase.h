class NarrowPhase : PerformanceMeasurementsHelper
{
public:
	NarrowPhase(int bodiesCount);

	const std::vector<Collision>& DetectCollisions(const std::vector<std::pair<int, int> >& overlapingList, const std::vector<IBody*>& bodies);

	int GetNumOfCollisions();
private:
	bool BallToBallCollisionTest(const Ball& ballA, const Ball& ballB, Collision& collision);
	bool BallToChainCollisionTest(const Ball& ballA, const Chain& ballB, Collision& collision);

	std::vector<Collision> collisionsList;

	int	m_collisionNumber;
};