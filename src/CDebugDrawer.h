#pragma once

class VisualizationWinow;

class CDebugDrawer : public IDebugDrawer
{
	enum pointsInCircle
	{
		k_pointsInCircle = 15
	};
public:
	CDebugDrawer(VisualizationWinow* window);
	void DrawCircle(const vector2df& position, float radius);
	void DrawLine(const vector2df& p1, const vector2df& p2);
	void DrawSequenceOfLines(const vector2df* p, int size);
	void DrawClosedSequenceOfLines(const vector2df* p, int size);
private:
	VisualizationWinow* m_window;
	std::vector<vector2df> m_points;
};