#include "StdAfx.h"
#include "vector2d.h"
#include "IDebugDrawer.h"
#include "CDebugDrawer.h"
#include "VisualizationWinow.h"

CDebugDrawer::CDebugDrawer(VisualizationWinow* window) : m_window(window)
{
}

void CDebugDrawer::DrawCircle(const vector2df& position, float radius)
{
	m_points.clear();
	for (int i = 0; i < k_pointsInCircle; ++i)
	{
		m_points.push_back(position + vector2df(cosf((2.0f * float(M_PI) / k_pointsInCircle) * i), sinf((2.0f * float(M_PI) / k_pointsInCircle) * i)) * radius);
	}
	DrawClosedSequenceOfLines(&m_points[0], m_points.size());
}

void CDebugDrawer::DrawLine(const vector2df& p1, const vector2df& p2)
{
	const unsigned char color[] = { 0xFF, 0xFF, 0xFF };
	m_window->DrawLine(p1.x, p1.y, p2.x, p2.y, color);
}

void CDebugDrawer::DrawSequenceOfLines(const vector2df* p, int size)
{
	m_points.clear();
	for (int i = 0; i < size; ++i)
	{
		m_points.push_back(p[i]);
	}
	const unsigned char color[] = { 0xFF, 0xFF, 0xFF };
	m_window->DrawSequenceOfLines(m_points, color);
}

void CDebugDrawer::DrawClosedSequenceOfLines(const vector2df* p, int size)
{
	DrawSequenceOfLines(p, size);
	DrawLine(p[size - 1], p[0]);
}
