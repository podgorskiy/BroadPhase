#define _USE_MATH_DEFINES
#include <cmath>
#include <cfloat>
#include "vector2d.h"
#include "aabb.h"
#include "IBody.h"
#include "Ball.h"
#include "IDebugDrawer.h"

Ball::Ball() : m_position(0, 0), m_angle(0),  m_radius(1.0f)
{
}

Ball::Ball(const vector2df& position, float rotation, float radius) :
	m_position(position),
	m_linearVelocity(vector2df(0, 0)),
	m_linearAcceleration(vector2df(0, 0)),
	m_angle(rotation),
	m_angularVelocity(0),
	m_angularAcceleration(0),
	m_radius(radius)
{
	m_invMass = 1.f / (float(M_PI) * radius * radius); //lets assume that material have a unit density
	m_invRestetution = 1.f / 0.5;
	m_invFriction = 1.f / 0.8;
	m_radiusSQ = m_radius * m_radius;
	m_invMomentumOfInertia = 2.0f * m_invMass / m_radiusSQ;
}

void Ball::UpdatePosition(float dt)
{
	m_position += m_linearVelocity * dt;
	m_linearVelocity += m_linearAcceleration * dt;
	m_linearAcceleration = vector2df(0.0f, -9.8f);

	m_angle += m_angularVelocity * dt;
	m_angularVelocity += m_angularAcceleration * dt;
	m_angularAcceleration = 0;
}

aabb2df Ball::GetAABB() const
{
	return aabb2df(m_position - vector2df(m_radius, m_radius), m_position + vector2df(m_radius, m_radius));
}

vector2df Ball::GetPosition() const
{
	return m_position;
}

void Ball::SetPosition(const vector2df& position)
{
	m_position = position;
}

float Ball::GetRotation() const
{
	return m_angle;
}

void Ball::SetRotation(float rotation)
{
	m_angle = rotation;
}

vector2df Ball::GetLinearVelocity()  const
{
	return m_linearVelocity;
}

void Ball::SetLinearVelocity(const vector2df& v)
{
	m_linearVelocity = v;
}

float Ball::GetAngularVelocity() const
{
	return m_angularVelocity;
}

void Ball::SetAngularVelocity(float w)
{
	m_angularVelocity = w;
}

float Ball::GetInvMass() const
{
	return m_invMass;
}

float Ball::GetInvMomentumOfI() const
{
	return m_invMomentumOfInertia;
}

float Ball::GetInvRestetution() const
{
	return m_invRestetution;
}

float Ball::GetInvFriction() const
{
	return m_invFriction;
}

float Ball::GetRadius() const
{
	return m_radius;
}

float Ball::GetSQRadius() const
{
	return m_radiusSQ;
}

Ball::BodyType Ball::GetBodyType()
{
	return BTypeBall;
}

void Ball::DebugDraw(IDebugDrawer* drawer)
{
	drawer->DrawCircle(m_position, m_radius);
	drawer->DrawLine(m_position, m_position + vector2df(m_angle) * m_radius);
}
