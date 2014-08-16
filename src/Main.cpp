#include "StdAfx.h"

#include "VisualizationWinow.h"
#include "vector2d.h"
#include "aabb.h"
#include "IDebugDrawer.h"
#include "CDebugDrawer.h"
#include "IBody.h"
#include "Ball.h"
#include "Chain.h"
#include "Profiler.h"
#include "IBroadPhase.h"
#include "BroadPhaseNaive.h"
#include "BroadPhaseSS.h"
#include "Collision.h"
#include "NarrowPhase.h"
#include "CollisionSolver.h"

#include "Main.h"

#include <tclap/CmdLine.h>


int main(int argc, char **argv)
{
	TCLAP::CmdLine cmd("The BroadPhase is a simple example of 2d physics engine that describes "
					   "the role of broad phase in physics simulations and collision detection "
					   "in particular. It demonstrates a naive implementation and sweep and "
					   "prune approach", ' ', "0.1");
	TCLAP::ValueArg<int> ballsCount(	"b", "ballscount",		"Number of balls in simulation. Default value is 200", false, 200, "int");
	TCLAP::SwitchArg sortAndSweep(		"s", "sortandsweep",	"Enables sort and sweep approach.", cmd, false);
	TCLAP::ValueArg<int> subStepsCount(	"u", "substeps",		"Number of substeps"
		"Count of physics frames per every draw frame. Default value is 10", false, 10, "int");
	cmd.add(ballsCount);
	cmd.add(subStepsCount);
	cmd.parse(argc, argv);

	Application* app = new Application(subStepsCount.getValue(), sortAndSweep.getValue());

	// Fill the world with some physics objects.
	app->CreateWorld(ballsCount.getValue());

	// Initialize internal modules, visualisation window, debug drawer and so on.
	app->Init();

	// Application run loop
	app->Run();

	delete app;

	return 0;
}

void Application::Init()
{
	// Constructing an object that handles broad phase.
	// BroadPhaseNaive - implements simple approach, testing for overlap all possible pairs
	// BroadPhaseSS - implements sweep and prune (sort and sweep) approach.
	if (m_sortAndSweep)
	{
		m_broadPhase = new BroadPhaseSS(m_bodies.size());
	}
	else
	{
		m_broadPhase = new BroadPhaseNaive(m_bodies.size());
	}

	m_narrowPhase = new NarrowPhase(m_bodies.size());
	m_collisionSolver = new CollisionSolver;

	// Initialisation of output window
	m_window.Init(ScreenWidth, ScreenHeight, 15.f, "Broad Phase");

	// Create debug drawer, that will handle drawing of all bodies.
	m_debugDrawer = new CDebugDrawer(&m_window);
}

void Application::Run()
{
	// Application run loop
	while (!m_window.GoingToClose())
	{
		for (int i = 0; i < m_subStepsCount; ++i)
		{
			// Update AABB
			m_broadPhase->UpdateAABBList(m_bodies);

			// Broad Phase
			const std::vector<std::pair<int, int> >& overlapingList = m_broadPhase->GenerateOverlapList();

			// Narrow phase
			const std::vector<Collision>& collisionList = m_narrowPhase->DetectCollisions(overlapingList, m_bodies);

			// ResolveCollisions
			m_collisionSolver->ResolveCollisions(collisionList, m_bodies);

			// Update motion of all bodies
			for (std::vector<IBody*>::const_iterator it = m_bodies.begin(); it != m_bodies.end(); ++it)
			{
				(*it)->UpdatePosition(1.0f / FrameRate / m_subStepsCount);
			}
		}

		// Draw all bodies
		m_window.BeginDraw();
		for (std::vector<IBody*>::const_iterator it = m_bodies.begin(); it != m_bodies.end(); ++it)
		{
			(*it)->DebugDraw(m_debugDrawer);
		}

		DrawFrameInfo();

		m_window.EndDraw();

		SleepMS(10);
	}
}

Application::~Application()
{
	delete m_broadPhase;
	delete m_narrowPhase;
	delete m_collisionSolver;
	delete m_debugDrawer;
}

void Application::DrawFrameInfo()
{
	std::vector<OutputDebug> outputBuffer;
	outputBuffer.push_back(OutputDebug("Broad phase time: %.1fus",
		PerformanceMeasurementsHelper::GetParameter(ProfileScopes::BroadPhase) / 1000.0f, wight));
	outputBuffer.push_back(OutputDebug("Narrow phase time: %.1fus",
		PerformanceMeasurementsHelper::GetParameter(ProfileScopes::NarrowPhase) / 1000.0f, wight));
	outputBuffer.push_back(OutputDebug("Collision solver time: %.1fus",
		PerformanceMeasurementsHelper::GetParameter(ProfileScopes::CollisionSolver) / 1000.0f, wight));
	outputBuffer.push_back(OutputDebug("Count of checks in broad phase: %d",
		PerformanceMeasurementsHelper::GetParameter(ProfileScopes::CountOfChecksInBroadPhase), wight));
	outputBuffer.push_back(OutputDebug("Count of potential collisions: %d",
		PerformanceMeasurementsHelper::GetParameter(ProfileScopes::CountOfPotentialCollisions), wight));
	outputBuffer.push_back(OutputDebug("Count of collisions occurred: %d",
		PerformanceMeasurementsHelper::GetParameter(ProfileScopes::CountOfCollisionsOccurred), wight));

	if (m_sortAndSweep)
	{
		outputBuffer.push_back(OutputDebug("Count of swaps occurred: %d",
			PerformanceMeasurementsHelper::GetParameter(ProfileScopes::CountOfSwaps), wight));
		outputBuffer.push_back(OutputDebug("Count of events dispatched: %d",
			PerformanceMeasurementsHelper::GetParameter(ProfileScopes::CountOfEventsDispatched), wight));
		outputBuffer.push_back(OutputDebug("UpdateEndPoint:  %.1fus",
			PerformanceMeasurementsHelper::GetParameter(ProfileScopes::UpdateEndPoint) / 1000.0f, wight));
		outputBuffer.push_back(OutputDebug("UpdateIntervals:  %.1fus",
			PerformanceMeasurementsHelper::GetParameter(ProfileScopes::UpdateIntervals) / 1000.0f, wight));
		outputBuffer.push_back(OutputDebug("ConsumeEvents:  %.1fus",
			PerformanceMeasurementsHelper::GetParameter(ProfileScopes::ConsumeEvents) / 1000.0f, wight));
		outputBuffer.push_back(OutputDebug("CreateOverlapingList:  %.1fus",
			PerformanceMeasurementsHelper::GetParameter(ProfileScopes::CreateOverlapingList) / 1000.0f, wight));
	}

	vector2d<int> textPosition(5, 20);
	for (std::vector<OutputDebug>::const_iterator it = outputBuffer.begin(); it != outputBuffer.end(); ++it, textPosition.y += 20)
	{
		m_window.DrawTextL(it->m_value().c_str(), textPosition.x, textPosition.y, it->m_color);
	}
}

void Application::CreateWorld(int ballsCount)
{
	// Let's fill bodies container with balls that have some random properties.
	for (int i = 0; i < ballsCount; ++i)
	{
		Ball* ball = new Ball(
			/// position of a ball will be in range of (x:[-2.5; 2.5], y:[-2.5; 2.5])
			vector2df( float(rand() % 30), float(rand() % 200)) / 10.0f - vector2df(0.0f, -5.0f),
			/// rotation in range of [0; pi]
			(rand() % 100) / 100.0f * float(M_PI),
			/// size in range of [0.2; 0.45]
			(rand() % 10) / 100.0f + 0.1f);
		m_bodies.push_back(ball);
	}

	// Let's create some chains that would limit move of balls.

	float verticesA[] = {
		-5.235f, -4.108f,
		-2.41f, -4.1081f,
		-2.40f, -4.5928f,
		-1.10f, -4.5920f,
		-1.11f, -5.095f,
		0.0f, -5.0955f,
		0.001f, -5.4611f,
		4.70f, -5.4612f
	};

	float verticesB[] = {
		-5.235f, -4.108f,
		-5.237f, 6.33f
	};

	float verticesC[] = {
		4.7f, -4.75f,
		4.71f, 5.5f
	};

	float verticesD[] = {
		-3.462f, -2.1707f,
		-2.109f, -2.171f,
		0.764f, -1.0922f,
		3.29f, -1.092f,
		3.289f, 0.000f
	};

	float verticesE[] = {
		0.0f, 0.0f,
		-4.8147293f, 1.1285802f,
		-5.2358660f, 1.9239694f
	};

	float verticesF[] = {
		-0.6725902f, 1.5262748f,
		2.0268912f, 2.3535493f,
		4.2721980f, 4.9812985f
	};

	Chain* chain = new Chain;
	chain->AddVertices(verticesA);
	m_bodies.push_back(chain);

	chain = new Chain;
	chain->AddVertices(verticesB);
	m_bodies.push_back(chain);

	chain = new Chain;
	chain->AddVertices(verticesC);
	m_bodies.push_back(chain);

	chain = new Chain;
	chain->AddVertices(verticesD);
	m_bodies.push_back(chain);

	chain = new Chain;
	chain->AddVertices(verticesE);
	m_bodies.push_back(chain);

	chain = new Chain;
	chain->AddVertices(verticesF);
	m_bodies.push_back(chain);
}
