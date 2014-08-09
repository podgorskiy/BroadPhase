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
#include "CollisionSolver.h"
#include "Profiler.h"

unsigned long long CollisionSolver::GetConumedTime()
{
	return m_consumedTime;
}

void CollisionSolver::ResolveCollisions(const std::vector<Collision>& collisionList, const std::vector<IBody*>& bodies)
{
#ifdef PROFILE
	TinyProfiler profiler;
#endif
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
#ifdef PROFILE
	m_consumedTime = profiler.GetTime();
#endif
}

float calc_impulse_p_p(float e, float sv, float nx, float ny, float ma, float mb, vector2df rap, vector2df rbp, float ia, float ib)
{
	float tmp = 1 / ma + 1 / mb + pow(rap.x*ny - rap.y*nx, 2) / ia + pow(rbp.x*ny - rbp.y*nx, 2) / ib;
	return -(1 + e)*sv / tmp;
}

void CollisionSolver::ResolveBallToBallCollision(Ball& ballA, Ball& ballB, const Collision& collision)
{
	vector2df velocityA = ballA.GetLinearVelocity();
	vector2df velocityB = ballB.GetLinearVelocity();
	float angularVelocityA = ballA.GetAngularVelocity();
	float angularVelocityB = ballB.GetAngularVelocity();
	vector2df posA = ballA.GetPosition();
	vector2df posB = ballB.GetPosition();
	float invMassA = ballA.GetInvMass();
	float invMassB = ballB.GetInvMass();

	vector2df relativeVelocity = velocityB + angularVelocityB * RotateCCW(collision.m_collisionPoint - posB)
		- (velocityA + angularVelocityA * RotateCCW(collision.m_collisionPoint - posA));
	float sv = collision.m_collisionNormal * relativeVelocity;

	// for a collision to occur relative normal velocity must be negative
	if (sv > -1e6f && sv < -1e-6)
	{
		vector2df rap = collision.m_collisionPoint - posA;
		vector2df rbp = collision.m_collisionPoint - posB;
		float imp = calc_impulse_p_p(0.5f, sv, collision.m_collisionNormal.x, collision.m_collisionNormal.y, 1.0f / invMassA,
			1.0f / invMassB, rap, rbp, 1.0f / ballA.GetInvMomentumOfI(),
			1.0f / ballB.GetInvMomentumOfI());
		float maxTangentImpulse = imp * 0.5f;

		velocityA -= imp * invMassA * collision.m_collisionNormal;
		velocityB += imp * invMassB * collision.m_collisionNormal;
		//angularVelocityA += imp * Cross(rap, collision.m_collisionNormal) * ballA.GetInvMomentumOfI();
		//angularVelocityB -= imp * Cross(rbp, collision.m_collisionNormal) * ballB.GetInvMomentumOfI();


		// friction
		vector2df tangent = RotateCCW(collision.m_collisionNormal);
		float tangentVelocity = tangent * relativeVelocity;
		float timp = calc_impulse_p_p(0.0f, tangentVelocity * 0.8f, tangent.x, tangent.y,
			1.0f / invMassA,
			1.0f / invMassB, rap, rbp,
			1.0f / ballA.GetInvMomentumOfI(),
			1.0f / ballB.GetInvMomentumOfI());

		timp = timp > maxTangentImpulse ? maxTangentImpulse : timp;
		timp = timp <-maxTangentImpulse ? -maxTangentImpulse : timp;

		vector2df tangentImpulse = timp * tangent;
		velocityA -= tangentImpulse * invMassA;
		velocityB += tangentImpulse * invMassB;
		float impMomentumA = Cross(rap, tangent);
		float impMomentumB = Cross(rbp, tangent);
		angularVelocityA -= timp * impMomentumA * ballA.GetInvMomentumOfI();
		angularVelocityB += timp * impMomentumB * ballB.GetInvMomentumOfI();

		ballA.SetLinearVelocity(velocityA);
		ballB.SetLinearVelocity(velocityB);
		ballA.SetAngularVelocity(angularVelocityA);
		ballB.SetAngularVelocity(angularVelocityB);
	}
	float rAB = ballA.GetRadius() + ballB.GetRadius();
	float d = (posB - posA).GetLength();
	posA -= collision.m_collisionNormal*((rAB - d) / invMassB / (1.0f / invMassA + 1.0f / invMassB));
	posB += collision.m_collisionNormal*((rAB - d) / invMassA / (1.0f / invMassA + 1.0f / invMassB));
	ballA.SetPosition(posA);
	ballB.SetPosition(posB);
}

void CollisionSolver::ResolveBallToChainCollision(Ball& ball, Chain& chain, const Collision& collision)
{
	vector2df velocity = ball.GetLinearVelocity();
	float angularVelocity = ball.GetAngularVelocity();
	vector2df position = ball.GetPosition();
	float invMass = ball.GetInvMass();

	vector2df relativeVelocity = -(velocity + angularVelocity * RotateCCW(collision.m_collisionPoint - position));
	float sv = collision.m_collisionNormal * relativeVelocity;
	if (sv > -1e6f && sv < 0)
	{
		float dv = -sv * (1.f + 0.8f);
		float impulseModule = dv / invMass;
		vector2df impulse = collision.m_collisionNormal * impulseModule;
		velocity -= impulse * invMass;

		// friction
		vector2df tangent = RotateCCW(collision.m_collisionNormal);
		float tangentVelocity = tangent * relativeVelocity;
		float dtv = tangentVelocity*0.9f;
		float totalTangentInvMass = ball.GetSQRadius() * ball.GetInvMomentumOfI();
		float tangentImpulseModule = dtv / totalTangentInvMass;
		float maxTangentImpulse = impulseModule / ball.GetInvFriction();
		tangentImpulseModule = tangentImpulseModule > maxTangentImpulse ? maxTangentImpulse : tangentImpulseModule;
		tangentImpulseModule = tangentImpulseModule <-maxTangentImpulse ? -maxTangentImpulse : tangentImpulseModule;
		vector2df tangentImpulse = tangent * tangentImpulseModule;
		float momentumOfImpulse = ball.GetRadius() * tangentImpulseModule;
		velocity += tangentImpulse * invMass;
		angularVelocity += momentumOfImpulse * ball.GetInvMomentumOfI();

		ball.SetLinearVelocity(velocity);
		ball.SetAngularVelocity(angularVelocity);
	}
	float d = (position - collision.m_collisionPoint).GetLength();
	position -= collision.m_collisionNormal*(ball.GetRadius() - d);
	ball.SetPosition(position);
}
