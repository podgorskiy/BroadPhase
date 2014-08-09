class CollisionSolver
{
public:
	CollisionSolver(){};

	void ResolveCollisions(const std::vector<Collision>& collisionList, const std::vector<IBody*>& bodies);
	unsigned long long GetConumedTime();
	
private:
	void ResolveBallToBallCollision(Ball& ballA, Ball& ballB, const Collision& collision);
	void ResolveBallToChainCollision(Ball& ball, Chain& chain, const Collision& collision);

	unsigned long long m_consumedTime;
};