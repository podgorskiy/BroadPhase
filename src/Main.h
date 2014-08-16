class Application
{
public:
	enum Constants
	{
		FrameRate = 60, // Lets have a constant time step, assuming that we have 60 frames per second.
		ScreenWidth = 1024,
		ScreenHeight = 768
	};

	Application(int subStepsCount, bool sortAndSweep):
		m_broadPhase(NULL), m_narrowPhase(NULL), m_collisionSolver(NULL), m_debugDrawer(NULL),
		m_subStepsCount(subStepsCount), m_sortAndSweep(sortAndSweep){};
	~Application();

	/// Initialize internal modules, visualisation window, debug drawer and so on.
	void Init();

	/// Fills the bodies container with some random balls and chains.
	void CreateWorld(int ballsCount);

	/// Print some frame info. Prints broad phase, narrow phase and collision solver time.
	void DrawFrameInfo();

	/// Application run loop
	void Run();

private:
	// Container for all bodies in simulation.
	std::vector<IBody*> m_bodies;

	IBroadPhase *		m_broadPhase;
	NarrowPhase*		m_narrowPhase;
	CollisionSolver*	m_collisionSolver;

	VisualizationWinow m_window;
	IDebugDrawer* m_debugDrawer;

	int m_subStepsCount;
	bool m_sortAndSweep;
};
