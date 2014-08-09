#define _USE_MATH_DEFINES
#include <cmath>
#include <cfloat>
#include <vector>
#include "vector2d.h"
#include "GeometryOperations2D.h"
#include "aabb.h"
#include "Collision.h"
#include "IBody.h"
#include "Ball.h"
#include "Chain.h"
#include "NarrowPhase.h"
#include "Profiler.h"

NarrowPhase::NarrowPhase(int bodiesCount)
{
	collisionsList.reserve(2 * bodiesCount);
}

unsigned long long NarrowPhase::GetConumedTime()
{
	return m_consumedTime;
}

int NarrowPhase::GetNumOfCollisions()
{
	return m_collisionNumber;
}

const std::vector<Collision>& NarrowPhase::DetectCollisions(const std::vector<std::pair<int, int> >& overlapingList, const std::vector<IBody*>& bodies)
{
#ifdef PROFILE
	TinyProfiler profiler;
#endif
	m_collisionNumber = 0;
	collisionsList.clear();
	for (std::vector<std::pair<int, int> >::const_iterator it = overlapingList.begin(); it != overlapingList.end(); ++it)
	{
		IBody* bodyA = bodies[it->first];
		IBody* bodyB = bodies[it->second];
		Collision collision;
		int collisionType = bodyA->GetBodyType() | bodyB->GetBodyType();
		switch (collisionType)
		{
			case IBody::BTypeBall:	//ball to ball collision
			{
				Ball* ballA = static_cast<Ball*>(bodyA);
				Ball* ballB = static_cast<Ball*>(bodyB);
				if (BallToBallCollisionTest(*ballA, *ballB, collision))
				{
					collision.m_indexA = it->first;
					collision.m_indexB = it->second;
					collisionsList.push_back(collision);
					m_collisionNumber++;
				}
			}
			break;
			case IBody::BTypeBall | IBody::BTypeChain: //ball to chain collision
			{
				int indexA = it->first;
				int indexB = it->second;
				if (bodyA->GetBodyType() == IBody::BTypeChain)
				{
					std::swap(bodyA, bodyB);
					std::swap(indexA, indexB);
				}
				Ball* ball = static_cast<Ball*>(bodyA);
				Chain* chain = static_cast<Chain*>(bodyB);
				if (BallToChainCollisionTest(*ball, *chain, collision))
				{
					collision.m_indexA = indexA;
					collision.m_indexB = indexB;
					collisionsList.push_back(collision);
					m_collisionNumber++;
				}
			}
			break;
		}
	}
#ifdef PROFILE
	m_consumedTime = profiler.GetTime();
#endif
	return collisionsList;
}

inline bool NarrowPhase::BallToBallCollisionTest(const Ball& ballA, const Ball& ballB, Collision& collision)
{
	vector2df centers(ballB.GetPosition() - ballA.GetPosition());
	float rAB = ballA.GetRadius() + ballB.GetRadius();
	if (centers.GetLengthSQ() < (rAB * rAB))
	{
		collision.m_collisionPoint = ballA.GetPosition() + 0.5f * centers * (ballB.GetRadius() / ballA.GetRadius());
		collision.m_collisionNormal = centers.GetNormalized();
		return true;
	}
	return false;
}

inline bool NarrowPhase::BallToChainCollisionTest(const Ball& ball, const Chain& chain, Collision& collision)
{
	const aabb2df ballAabb = ball.GetAABB();
	const vector2df ballCenter = ball.GetPosition();
	const std::vector<vector2df>& vertices = chain.GetVertices();
	std::vector<vector2df>::const_iterator lastVertex = --vertices.end();
	for (std::vector<vector2df>::const_iterator it = vertices.begin(); it != lastVertex; ++it)
	{
		const vector2df& p1 = *it;
		const vector2df& p2 = *(it + 1);
		aabb2df chainSegmentAabb(p1);
		chainSegmentAabb.AddPoint(p2);
		if (Overlapping(ballAabb, chainSegmentAabb))
		{
			float a, b, c;
			GetLineCoefficients(p1, p2, a, b, c);
			vector2df crossPoint;
			GetClothestPoint(a, b, c, ballCenter, crossPoint);
			if (IsPointBelongsToSegment<float, vector2df>(p1, p2, crossPoint))
			{
				vector2df dir = crossPoint - ballCenter;
				float distanceSQ = dir.GetLengthSQ();
				if (distanceSQ < ball.GetSQRadius())
				{
					collision.m_collisionNormal = dir.GetNormalized();
					collision.m_collisionPoint = crossPoint;
					return true;
				}
			}
			else
			{
				float dA = (ballCenter - p1).GetLengthSQ();
				if (dA < ball.GetSQRadius())
				{
					collision.m_collisionNormal = (p1 - ballCenter).GetNormalized();
					collision.m_collisionPoint = p1;
					return true;
				}
				float dB = (ballCenter - p2).GetLengthSQ();
				if (dB < ball.GetSQRadius())
				{
					collision.m_collisionNormal = (p2 - ballCenter).GetNormalized();
					collision.m_collisionPoint = p2;
					return true;
				}
			}
		}
	}
	return false;
}
