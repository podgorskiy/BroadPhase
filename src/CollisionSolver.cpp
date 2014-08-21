#include "StdAfx.h"
#include "vector2d.h"
#include "GeometryOperations2D.h"
#include "aabb.h"
#include "Collision.h"
#include "IBody.h"
#include "Ball.h"
#include "Chain.h"
#include "Profiler.h"
#include "CollisionSolver.h"

void CollisionSolver::ResolveCollisions(const std::vector<Collision>& collisionList, const std::vector<IBody*>& bodies)
{
	TimeProfiler broadPhaseTime(this, ProfileScopes::CollisionSolver);

	for (std::vector<Collision>::const_iterator it = collisionList.begin(); it != collisionList.end(); ++it)
	{
		IBody* bodyA = bodies[it->m_indexA];
		IBody* bodyB = bodies[it->m_indexB];

		int collisionType = bodyA->GetBodyType() | bodyB->GetBodyType();
		switch (collisionType)
		{
			case IBody::BTypeBall:	//ball to ball collision
			{
				Ball* ballA = static_cast<Ball*>(bodyA);
				Ball* ballB = static_cast<Ball*>(bodyB);
				ResolveBallToBallCollision(*ballA, *ballB, *it);
			}
			break;
			case IBody::BTypeBall | IBody::BTypeChain: //ball to chain collision
			{
				Ball* ball = static_cast<Ball*>(bodyA);
				Chain* chain = static_cast<Chain*>(bodyB);
				ResolveBallToChainCollision(*ball, *chain, *it);
			}
			break;
		}
	}
}

void CollisionSolver::TwoDynamicBodiesImpulseExchange(IBody& A, IBody& B, const Collision& collision)
{
	vector2df velocityA = A.GetLinearVelocity();
	vector2df velocityB = B.GetLinearVelocity();
	float angularVelocityA = A.GetAngularVelocity();
	float angularVelocityB = B.GetAngularVelocity();
	vector2df posA = A.GetPosition();
	vector2df posB = B.GetPosition();
	float invMassA = A.GetInvMass();
	float invMassB = B.GetInvMass();
	float restetution = 1.0f / (A.GetInvRestetution() + B.GetInvRestetution());
	float friction = 1.0f / (A.GetInvFriction() + B.GetInvFriction());

	vector2df relativeVelocity = velocityB + angularVelocityB * RotateCCW(collision.m_collisionPoint - posB)
		- (velocityA + angularVelocityA * RotateCCW(collision.m_collisionPoint - posA));
	float sv = collision.m_collisionNormal * relativeVelocity;

	// for a collision to occur relative normal velocity must be negative
	if (sv > -1e6f && sv < -1e-6)
	{
		vector2df rap = collision.m_collisionPoint - posA;
		vector2df rbp = collision.m_collisionPoint - posB;
		float imp = CalcImpulseExchange(restetution, sv, collision.m_collisionNormal.x, collision.m_collisionNormal.y, invMassA,
			invMassB, rap, rbp, A.GetInvMomentumOfI(), B.GetInvMomentumOfI());
		float maxTangentImpulse = imp * friction;

		velocityA -= imp * invMassA * collision.m_collisionNormal;
		velocityB += imp * invMassB * collision.m_collisionNormal;
		angularVelocityA += imp * Cross(rap, collision.m_collisionNormal) * A.GetInvMomentumOfI();
		angularVelocityB -= imp * Cross(rbp, collision.m_collisionNormal) * B.GetInvMomentumOfI();

		// friction
		vector2df tangent = RotateCCW(collision.m_collisionNormal);
		float tangentVelocity = tangent * relativeVelocity;
		float timp = CalcImpulseExchange(0.0f, tangentVelocity * 0.8f, tangent.x, tangent.y,
			invMassA, invMassB, rap, rbp, A.GetInvMomentumOfI(), B.GetInvMomentumOfI());

		timp = timp > maxTangentImpulse ? maxTangentImpulse : timp;
		timp = timp <-maxTangentImpulse ? -maxTangentImpulse : timp;

		vector2df tangentImpulse = timp * tangent;
		velocityA -= tangentImpulse * invMassA;
		velocityB += tangentImpulse * invMassB;
		float impMomentumA = Cross(rap, tangent);
		float impMomentumB = Cross(rbp, tangent);
		angularVelocityA -= timp * impMomentumA * A.GetInvMomentumOfI();
		angularVelocityB += timp * impMomentumB * B.GetInvMomentumOfI();

		A.SetLinearVelocity(velocityA);
		B.SetLinearVelocity(velocityB);
		A.SetAngularVelocity(angularVelocityA);
		B.SetAngularVelocity(angularVelocityB);
	}
}

void CollisionSolver::DynamicAndStaticBodiesImpulseExchange(IBody& dynamicBody, const IBody& staticBody, const Collision& collision)
{
	vector2df velocity = dynamicBody.GetLinearVelocity();
	float angularVelocity = dynamicBody.GetAngularVelocity();
	vector2df d_pos = dynamicBody.GetPosition();
	vector2df s_pos = staticBody.GetPosition();
	float invMass = dynamicBody.GetInvMass();
	float restetution = 1.0f / dynamicBody.GetInvRestetution();
	float friction = 1.0f / dynamicBody.GetInvFriction();

	vector2df relativeVelocity = -(velocity + angularVelocity * RotateCCW(collision.m_collisionPoint - d_pos));
	float sv = collision.m_collisionNormal * relativeVelocity;

	// for a collision to occur relative normal velocity must be negative
	if (sv > -1e6f && sv < -1e-6)
	{
		vector2df rap = collision.m_collisionPoint - d_pos;
		vector2df rbp = collision.m_collisionPoint - s_pos;
		float imp = CalcDegenerateImpulseExchange(restetution, sv, collision.m_collisionNormal.x, collision.m_collisionNormal.y, 
			invMass, rap, dynamicBody.GetInvMomentumOfI());
		float maxTangentImpulse = imp * friction;

		velocity -= imp * invMass * collision.m_collisionNormal;
		angularVelocity += imp * Cross(rap, collision.m_collisionNormal) * dynamicBody.GetInvMomentumOfI();

		// friction
		vector2df tangent = RotateCCW(collision.m_collisionNormal);
		float tangentVelocity = tangent * relativeVelocity;
		float timp = CalcDegenerateImpulseExchange(0.0f, tangentVelocity * 0.8f, tangent.x, tangent.y,
			invMass, rap, dynamicBody.GetInvMomentumOfI());

		timp = timp > maxTangentImpulse ? maxTangentImpulse : timp;
		timp = timp <-maxTangentImpulse ? -maxTangentImpulse : timp;

		vector2df tangentImpulse = timp * tangent;
		velocity -= tangentImpulse * invMass;
		float impMomentumA = Cross(rap, tangent);
		float impMomentumB = Cross(rbp, tangent);
		angularVelocity -= timp * impMomentumA * dynamicBody.GetInvMomentumOfI();

		dynamicBody.SetLinearVelocity(velocity);
		dynamicBody.SetAngularVelocity(angularVelocity);
	}
}

void CollisionSolver::ResolveBallToBallCollision(Ball& ballA, Ball& ballB, const Collision& collision)
{
	TwoDynamicBodiesImpulseExchange(ballA, ballB, collision);

	vector2df posA = ballA.GetPosition();
	vector2df posB = ballB.GetPosition();

	float rAB = ballA.GetRadius() + ballB.GetRadius();
	float d = (posB - posA).GetLength();

	float massA = 1.0f / ballA.GetInvMass();
	float massB = 1.0f / ballB.GetInvMass();

	float commonPart = (rAB - d) / (massA + massB);
	posA -= collision.m_collisionNormal * commonPart * massA;
	posB += collision.m_collisionNormal * commonPart * massB;
	ballA.SetPosition(posA);
	ballB.SetPosition(posB);
}

void CollisionSolver::ResolveBallToChainCollision(Ball& ball, Chain& chain, const Collision& collision)
{
	DynamicAndStaticBodiesImpulseExchange(ball, chain, collision);

	vector2df position = ball.GetPosition();
	float d = (position - collision.m_collisionPoint).GetLength();
	position -= collision.m_collisionNormal*(ball.GetRadius() - d);
	ball.SetPosition(position);
}

inline float CollisionSolver::CalcImpulseExchange(float e, float sv, float nx, float ny, float ima, float imb,
	vector2df rap, vector2df rbp, float iia, float iib)
{
	float tmp = ima + imb + pow(rap.x*ny - rap.y*nx, 2) * iia + pow(rbp.x*ny - rbp.y*nx, 2) * iib;
	return -(1 + e)*sv / tmp;
}

inline float CollisionSolver::CalcDegenerateImpulseExchange(float e, float sv, float nx, float ny, float im,
	vector2df rp, float ii)
{
	float tmp = im + pow(rp.x*ny - rp.y*nx, 2) * ii;
	return -(1 + e)*sv / tmp;
}