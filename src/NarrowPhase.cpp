#include "StdAfx.h"
#include "vector2d.h"
#include "GeometryOperations2D.h"
#include "aabb.h"
#include "CStyleArray.h"
#include "Collision.h"
#include "IBody.h"
#include "Ball.h"
#include "Chain.h"
#include "Profiler.h"
#include "NarrowPhase.h"

NarrowPhase::NarrowPhase(int bodiesCount)
{
	collisionsList.reserve(2 * bodiesCount);
}

int NarrowPhase::GetNumOfCollisions()
{
	return m_collisionNumber;
}

const std::vector<Collision>& NarrowPhase::DetectCollisions(const CStyleArray<int>& overlapingList, const std::vector<IBody*>& bodies)
{
	TimeProfiler broadPhaseTime(this, ProfileScopes::NarrowPhase);

	collisionsList.clear();
	for (int it = 0; it != overlapingList.size(); ++it)
	{
		if (overlapingList[it] == -1)
		{
			continue;
		}
		int index0 = overlapingList[it] & 0x0000FFFF;
		int index1 = overlapingList[it] >> 16;
		IBody* bodyA = bodies[index0];
		IBody* bodyB = bodies[index1];
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
					collision.m_indexA = index0;
					collision.m_indexB = index1;
					collisionsList.push_back(collision);
					IncValue(ProfileScopes::CountOfCollisionsOccurred);
				}
			}
			break;
			case IBody::BTypeBall | IBody::BTypeChain: //ball to chain collision
			{
				int indexA = index0;
				int indexB = index1;
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
					IncValue(ProfileScopes::CountOfCollisionsOccurred);
				}
			}
			break;
		}
	}

	SubmitValue(ProfileScopes::CountOfCollisionsOccurred);
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
