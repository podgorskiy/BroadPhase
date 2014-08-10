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
#include "Collision.h"
#include "NarrowPhase.h"
#include "CollisionSolver.h"


/// a function that fills the bodies container with some random balls and chains
void CreateWorld(std::vector<IBody*>& bodies);


int main(int argc,char **argv)
{
	// Container for all bodies in simulation.
	std::vector<IBody*> bodies;

	// fill the container with some bodies.
	CreateWorld(bodies);

	IBroadPhase * broadPhase			= new BroadPhaseNaive(bodies.size());
	NarrowPhase* narrowPhase			= new NarrowPhase(bodies.size());
	CollisionSolver* collisionSolver	= new CollisionSolver;

	// Initialisation of output window
	VisualizationWinow window;
	window.Init(1024,768,15.f,"Broad Phase");

	// Create debug drawer, that will handle drawing of all bodies.
	IDebugDrawer* debugDrawer = new CDebugDrawer(&window);

	// count of physics frames per every draw frame
	int subStepCount = 10;

	// Application main loop
	while (!window.GoingToClose())
	{
		for (int i = 0; i < subStepCount; ++i)
		{
			// Update AABB
			broadPhase->UpdateAABBList(bodies);

			// Broad Phase
			const std::vector<std::pair<int, int> >& overlapingList = broadPhase->GenerateOverlapList();

			// Narrow phase
			const std::vector<Collision>& collisionList = narrowPhase->DetectCollisions(overlapingList, bodies);

			// ResolveCollisions
			collisionSolver->ResolveCollisions(collisionList, bodies);

			// Update motion of all bodies
			for (std::vector<IBody*>::const_iterator it = bodies.begin(); it != bodies.end(); ++it)
			{
				(*it)->UpdatePosition(1.0f / 60 / subStepCount); // Lets have a constant time step, assuming that we have 60 frames per second
			}
		}

		// Draw all bodies
		window.BeginDraw();
		for (std::vector<IBody*>::const_iterator it = bodies.begin(); it != bodies.end(); ++it)
		{
			(*it)->DebugDraw(debugDrawer);
		}

		// Output of debug information
		std::vector<OutputDebug> outputBuffer;
		outputBuffer.push_back(OutputDebug("Broad phase time: %.1fus",
			PerformanceMeasurementsHelper::GetParameter(ProfileScopes::BroadPhase) / 1000.0f, wight));
		outputBuffer.push_back(OutputDebug("Narrow phase time: %.1fus",
			PerformanceMeasurementsHelper::GetParameter(ProfileScopes::NarrowPhase) / 1000.0f, wight));
		outputBuffer.push_back(OutputDebug("Collision solver time: %.1fus",
			PerformanceMeasurementsHelper::GetParameter(ProfileScopes::CollisionSolver) / 1000.0f, wight));
		outputBuffer.push_back(OutputDebug("Number of checks in broad phase: %d",
			PerformanceMeasurementsHelper::GetParameter(ProfileScopes::CountOfChecksInBroadPhase), wight));
		outputBuffer.push_back(OutputDebug("Number of potential collisions: %d",
			PerformanceMeasurementsHelper::GetParameter(ProfileScopes::CountOfPotentialCollisions), wight));
		outputBuffer.push_back(OutputDebug("Number of collisions occurred: %d",
			PerformanceMeasurementsHelper::GetParameter(ProfileScopes::CountOfCollisionsOccurred), wight));

		vector2d<int> textPosition(5, 20);
		for (std::vector<OutputDebug>::const_iterator it = outputBuffer.begin(); it != outputBuffer.end(); ++it, textPosition.y+=20)
		{
			window.DrawTextL(it->m_value().c_str(), textPosition.x, textPosition.y, it->m_color);
		}

		window.EndDraw();

		SleepMS(10);
	}

	delete broadPhase;
	delete narrowPhase;
	delete collisionSolver;
	return 0;
}

void CreateWorld(std::vector<IBody*>& bodies)
{
	// Let's fill bodies container with balls that have some random properties.
	for (int i = 0; i < 100; ++i)
	{
		Ball* ball = new Ball(
			/// position of a ball will be in range of (x:[-2.5; 2.5], y:[-2.5; 2.5])
			vector2df( float(rand() % 30), float(rand() % 100)) / 10.0f - vector2df(0.0f, -5.0f),
			/// rotation in range of [0; pi]
			(rand() % 100) / 100.0f * float(M_PI),
			/// size in range of [0.2; 0.45]
			(rand() % 10) / 100.0f + 0.1f);
		bodies.push_back(ball);
	}

	// Let's create some chains that would limit move of balls.
	Chain* chain = new Chain;
	chain->AddVertex(vector2df(-5.235f,		-4.108f));
	chain->AddVertex(vector2df(-2.41f,		-4.1081f));
	chain->AddVertex(vector2df(-2.40f,		-4.5928f));
	chain->AddVertex(vector2df(-1.10f,		-4.5920f));
	chain->AddVertex(vector2df(-1.11f,		-5.095f));
	chain->AddVertex(vector2df( 0.0f,		-5.0955f));
	chain->AddVertex(vector2df( 0.001f,		-5.4611f));
	chain->AddVertex(vector2df( 4.70f,		-5.4612f));
	bodies.push_back(chain);

	chain = new Chain;
	chain->AddVertex(vector2df(-5.235f,		-4.108f));
	chain->AddVertex(vector2df(-5.237f,		 6.33f));
	bodies.push_back(chain);

	chain = new Chain;
	chain->AddVertex(vector2df(4.7f,		-4.75f));
	chain->AddVertex(vector2df(4.71f,		 5.5f));
	bodies.push_back(chain);

	chain = new Chain;
	chain->AddVertex(vector2df(-3.462f,		-2.1707f));
	chain->AddVertex(vector2df(-2.109f,		-2.171f));
	chain->AddVertex(vector2df(0.764f,		-1.0922f));
	chain->AddVertex(vector2df(3.29f,		-1.092f));
	chain->AddVertex(vector2df(3.289f,		 0.000f));
	bodies.push_back(chain);

	chain = new Chain;
	chain->AddVertex(vector2df( 0.0f,		 0.0f));
	chain->AddVertex(vector2df(-4.8147293f,	 1.1285802f));
	chain->AddVertex(vector2df(-5.2358660f,	 1.9239694f));
	bodies.push_back(chain);

	chain = new Chain;
	chain->AddVertex(vector2df(-0.6725902f,	 1.5262748f));
	chain->AddVertex(vector2df( 2.0268912f,  2.3535493f));
	chain->AddVertex(vector2df( 4.2721980f,  4.9812985f));
	bodies.push_back(chain);







}
