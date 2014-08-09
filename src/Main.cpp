#define _USE_MATH_DEFINES
#include <cmath>
#include <cstdlib>
#include <vector>
#include <sstream>
#include "VisualizationWinow.h"
#include <cfloat>
#include "vector2d.h"
#include "GeometryOperations2D.h"
#include "aabb.h"
#include "IDebugDrawer.h"
#include "CDebugDrawer.h"
#include "IBody.h"
#include "Ball.h"
#include "Chain.h"
#include "BroadPhase.h"
#include "Collision.h"
#include "NarrowPhase.h"
#include "CollisionSolver.h"
#include "Profiler.h"


/// a function that fills the bodies container with some random balls and chains
void CreateWorld(std::vector<IBody*>& bodies);


int main(int argc,char **argv)
{
	// Container for all bodies in simulation.
	std::vector<IBody*> bodies;

	// fill the container with some bodies.
	CreateWorld(bodies);

	BroadPhase * broadPhase				= new BroadPhase(bodies.size());
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

		// Output debug information
		unsigned long long broadPhasetime = broadPhase->GetConumedTime();
		std::ostringstream ss;
		ss << "Broad phase time: " << broadPhasetime / 1000 << broadPhasetime % 1000;
		std::string bdroadPhaseInfo(ss.str());
		window.DrawTextL(bdroadPhaseInfo.c_str(), 5, 50, wight);

		// Output debug information
		unsigned long long narrowPhaseTime = narrowPhase->GetConumedTime();
		std::ostringstream ss2;
		ss2 << "Narrow phase time: " << narrowPhaseTime / 1000 << narrowPhaseTime % 1000;
		std::string narrowPhaseInfo(ss2.str());
		window.DrawTextL(narrowPhaseInfo.c_str(), 5, 70, wight);

		// Output debug information
		unsigned long long collisionSolverTime = collisionSolver->GetConumedTime();
		std::ostringstream ss3; 
		ss3 << "Collision solver time: " << collisionSolverTime / 1000 << collisionSolverTime % 1000;
		std::string collisionSolverInfo(ss3.str());
		window.DrawTextL(collisionSolverInfo.c_str(), 5, 90, wight);

		// Output debug information
		int numOfChecks = broadPhase->GetNumOfChecks();
		std::ostringstream ss4;
		ss4 << "Number of checks in broad phase: " << numOfChecks;
		std::string numOfChecksInfo(ss4.str());
		window.DrawTextL(numOfChecksInfo.c_str(), 5, 130, wight);

		// Output debug information
		int numOfPotentialCollisions = broadPhase->GetNumOfPotentialCollisions();
		std::ostringstream ss5;
		ss5 << "Number of potential collisions: " << numOfPotentialCollisions;
		std::string numOfPotentialCollisionsInfo(ss5.str());
		window.DrawTextL(numOfPotentialCollisionsInfo.c_str(), 5, 150, wight);

		// Output debug information
		int numOfCollisions = narrowPhase->GetNumOfCollisions();
		std::ostringstream ss6;
		ss6 << "Number of collisions occurred: " << numOfCollisions;
		std::string numOfCollisionsInfo(ss6.str());
		window.DrawTextL(numOfCollisionsInfo.c_str(), 5, 170, wight);

		window.EndDraw();
		//_sleep(10);
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
