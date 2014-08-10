#pragma once

#define PROFILE

#define OUTPUT_AVERAGE_OF   (10 * 60 / 4)

struct ProfileScopes
{
	enum Scope
	{
		BroadPhase,
		NarrowPhase,
		CollisionSolver,
		CountOfChecksInBroadPhase,
		CountOfPotentialCollisions,
		CountOfCollisionsOccurred,

		CountOfProfileScopes
	};
};
