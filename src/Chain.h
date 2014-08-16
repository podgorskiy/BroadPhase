#pragma once

class IDebugDrawer;

class Chain : public IBody
{
public:
	Chain();
	virtual ~Chain();

	virtual aabb2df		GetAABB() const;
	virtual vector2df	GetPosition() const						{ return vector2df(); };
	virtual float		GetRotation() const						{ return 0; };
	virtual void		SetPosition(const vector2df& position)	{};
	virtual void		SetRotation(const float rotation)		{};
	virtual vector2df	GetLinearVelocity()	const				{ return vector2df(); };
	virtual void		SetLinearVelocity(const vector2df& v)   {};
	virtual void		SetAngularVelocity(float w)				{};
	virtual float		GetAngularVelocity() const				{ return 0; };
	virtual float		GetMass() const							{ return 0; };
	virtual float		GetInvMass() const						{ return 0; };
	virtual float		GetInvMomentumOfI() const				{ return 0; };
	virtual float		GetInvRestetution() const				{ return 0; };
	virtual float		GetInvFriction() const					{ return 0; };
	virtual void		UpdatePosition(float dt)				{};
	virtual BodyType	GetBodyType();
	virtual void		DebugDraw(IDebugDrawer* drawer);

	void				AddVertex(const vector2df& v);
	void				AddVertices(const vector2df* v, int count);
	void				AddVertices(const float* v, int count);

	template <unsigned N>
	void				AddVertices(const vector2df (&v)[N])
	{
		AddVertices(&v[0], N);
	};
	template <unsigned N>
	void				AddVertices(const float (&v)[N])
	{
		AddVertices(&v[0], N);
	};

	const std::vector<vector2df>& GetVertices() const;

private:
	aabb2df					m_aabb; // Chains are static, let's keep the aabb.
	std::vector<vector2df>	m_vertices;
};
