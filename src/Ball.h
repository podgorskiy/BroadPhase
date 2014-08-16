#pragma once

class IDebugDrawer;

class Ball : public IBody
{
public:
	Ball();
	Ball(const vector2df& position, float rotation, float radius);
	virtual ~Ball() {};

	virtual aabb2df		GetAABB() const;

	virtual vector2df	GetPosition() const;
	virtual void		SetPosition(const vector2df& position);
	virtual float		GetRotation() const;
	virtual void		SetRotation(const float rotation);

	virtual vector2df	GetLinearVelocity() const;
	virtual void		SetLinearVelocity(const vector2df& v);
	virtual float		GetAngularVelocity() const;
	virtual void		SetAngularVelocity(float w);

	virtual float		GetRadius() const;
	virtual float		GetSQRadius() const;

	virtual float		GetInvMass() const;
	virtual float		GetInvMomentumOfI() const;
	virtual float		GetInvRestetution() const;
	virtual float		GetInvFriction() const;

	virtual void		UpdatePosition(float dt);
	virtual BodyType	GetBodyType() const;
	virtual void		DebugDraw(IDebugDrawer* drawer) const;

private:
	vector2df	m_position;
	vector2df	m_linearVelocity;
	vector2df	m_linearAcceleration;

	float		m_angle;
	float		m_angularVelocity;
	float		m_angularAcceleration;

	float		m_radius;
	float		m_radiusSQ;

	float		m_invMass;
	float		m_invMomentumOfInertia;
	float		m_invRestetution;
	float		m_invFriction;

	aabb2df		m_aabb;
};
