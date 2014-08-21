class CollisionSolver : PerformanceMeasurementsHelper
{
public:
	CollisionSolver(){};

	void ResolveCollisions(const std::vector<Collision>& collisionList, const std::vector<IBody*>& bodies);
	
private:
	void TwoDynamicBodiesImpulseExchange(IBody& A, IBody& B, const Collision& collision);
	void DynamicAndStaticBodiesImpulseExchange(IBody& dynamicBody, const IBody& staticBody, const Collision& collision);
	void ResolveBallToBallCollision(Ball& ballA, Ball& ballB, const Collision& collision);
	void ResolveBallToChainCollision(Ball& ball, Chain& chain, const Collision& collision);

	float CalcImpulseExchange(float e, float sv, float nx, float ny, float ima, float imb,
		vector2df rap, vector2df rbp, float iia, float iib);

	float CalcDegenerateImpulseExchange(float e, float sv, float nx, float ny, float im,
		vector2df rp, float ii);
};