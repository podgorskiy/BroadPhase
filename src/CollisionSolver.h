class CollisionSolver : PerformanceMeasurementsHelper
{
public:
	CollisionSolver(){};

	void ResolveCollisions(const std::vector<Collision>& collisionList, const std::vector<IBody*>& bodies);
	
private:
	void ResolveBallToBallCollision(Ball& ballA, Ball& ballB, const Collision& collision);
	void ResolveBallToChainCollision(Ball& ball, Chain& chain, const Collision& collision);
};