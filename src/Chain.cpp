#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <cfloat>
#include "vector2d.h"
#include "aabb.h"
#include "IBody.h"
#include "Chain.h"
#include "IDebugDrawer.h"

Chain::Chain()
{
}

Chain::~Chain() {};

aabb2df	Chain::GetAABB() const
{
	return m_aabb;
}

IBody::BodyType Chain::GetBodyType()
{
	return IBody::BTypeChain;
}

void Chain::DebugDraw(IDebugDrawer* drawer)
{
	drawer->DrawSequenceOfLines(&m_vertices[0], m_vertices.size());
}

void Chain::AddVertex(const vector2df& v)
{
	m_vertices.push_back(v);
	m_aabb.AddPoint(v);
}

const std::vector<vector2df>& Chain::GetVertices() const
{
	return m_vertices;
}
