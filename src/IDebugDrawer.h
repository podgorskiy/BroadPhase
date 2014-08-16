#pragma once

class IDebugDrawer
{
public:
    virtual ~IDebugDrawer(){};
	virtual void DrawCircle(const vector2df& position, float radius) = 0;
	virtual void DrawLine(const vector2df& p1, const vector2df& p2) = 0;
	virtual void DrawSequenceOfLines(const vector2df* p, int size) = 0;
	virtual void DrawClosedSequenceOfLines(const vector2df* p, int size) = 0;
};
