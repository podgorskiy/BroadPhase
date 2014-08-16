#include "StdAfx.h"
#include "vector2d.h"
#include "aabb.h"
#include "IBody.h"
#include "Chain.h"
#include "IDebugDrawer.h"

Chain::Chain() {}

Chain::~Chain() {}

aabb2df	Chain::GetAABB() const
{
	return m_aabb;
}

IBody::BodyType Chain::GetBodyType() const
{
	return IBody::BTypeChain;
}

void Chain::DebugDraw(IDebugDrawer* drawer) const
{
	drawer->DrawSequenceOfLines(&m_vertices[0], m_vertices.size());
}

void Chain::AddVertex(const vector2df& v)
{
	m_vertices.push_back(v);
	m_aabb.AddPoint(v);
}

void Chain::AddVertices(const vector2df* v, int count)
{
	for (int i = 0; i < count; ++i)
	{
		m_vertices.push_back(v[i]);
		m_aabb.AddPoint(v[i]);
	}
}

void Chain::AddVertices(const float* v, int count)
{
    assert(count % 2 == 0);
	for (int i = 0; i < count / 2; ++i)
	{
	    vector2df p(v[2 * i], v[2 * i + 1]);
		m_vertices.push_back(p);
		m_aabb.AddPoint(p);
	}
}

const std::vector<vector2df>& Chain::GetVertices() const
{
	return m_vertices;
}
