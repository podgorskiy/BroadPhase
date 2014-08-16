#pragma once

class IDebugDrawer;

class IBody
{
public:
	enum BodyType
	{
		BTypeNone = 0,
		BTypeChain = 1,
		BTypeBall = BTypeChain << 1
	};

	IBody() {};
	virtual ~IBody() {};

	virtual aabb2df		GetAABB() const = 0;
	
	virtual vector2df	GetPosition() const = 0;
	virtual void		SetPosition(const vector2df& position) = 0;
	virtual float		GetRotation() const = 0;
	virtual void		SetRotation(const float rotation) = 0;
	
	virtual vector2df	GetLinearVelocity() const = 0;
	virtual void		SetLinearVelocity(const vector2df& v) = 0;
	virtual float		GetAngularVelocity() const = 0;
	virtual void		SetAngularVelocity(float w) = 0;

	virtual float		GetInvMass() const = 0;
	virtual float		GetInvMomentumOfI() const = 0;
	virtual float		GetInvRestetution() const = 0;
	virtual float		GetInvFriction() const = 0;

	virtual void		UpdatePosition(float dt) = 0;
	virtual BodyType	GetBodyType() const = 0;
	virtual void		DebugDraw(IDebugDrawer* drawer) const = 0;
};