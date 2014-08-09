#pragma once

/// Get coefficients of line equation from two points.
template<typename F, typename T>
inline void GetLineCoefficients(const T& p1, const T& p2, F& a, F& b, F& c)
{
	a = p1.y - p2.y;
	b = p2.x - p1.x;
	c = p1.x*p2.y - p2.x*p1.y;
}

/// Get cross point of line and perpendicular to it from other point.
template<typename F, typename T>
inline void GetClothestPoint(const F a, const F b, const F c, const T& p, T& cp)
{
	const F c1 = b * p.x - a * p.y;
	const F invDet = F(1.0) / (a * a + b * b);
	cp.Set(-(a * c - b * c1) * invDet, -(a * c1 + b * c) * invDet);
}

/// Check if the point belongs to the segment.
template<typename F, typename T>
inline bool IsPointBelongsToSegment(const T& p1, const T& p2, const T& c)
{
	return	(
		(((p2.x >= c.x) && (c.x >= p1.x)) ||
		((p2.x <= c.x) && (c.x <= p1.x))) &&
		(((p2.y >= c.y) && (c.y >= p1.y)) ||
		((p2.y <= c.y) && (c.y <= p1.y)))
		);
}

/// Get a cross point of two lines, that are represented by the coefficients of line equations.
template<typename F, typename T>
inline void GetCrossPoint(F a1, F b1, F c1, F a2, F b2, F c2, T& cp)
{
	float tmp = a1 * b2 - a2 * b1;
	if (tmp == 0.0f)
	{
		cp.Set(FLT_MAX, FLT_MAX);
	}
	else
	{
		float inv = F(1.0) / tmp;
		cp.Set((b1 * c2 - b2 * c1) * inv, (a2 * c1 - a1 * c2) * inv);
	}
}

/// Get a cross point of two lines, that are represented by two pair of points.
template<typename F, typename T>
inline void GetCrossPoint(const T& l1p1, const T& l1p2, const T& l2p1, const T& l2p2, T& cp)
{
	F a1, b1, c1, a2, b2, c2;
	GetLineCoefficients(a1, b1, c1, l1p1, l1p2);
	GetLineCoefficients(a2, b2, c2, l2p1, l2p2);
	GetCrossPoint<F, T>(a1, b1, c1, a2, b2, c2, cp);
}

/// Check if two segments intercepts.
template<typename F, typename T>
inline bool CheckLinesForInterception(const T& l1p1, const T& l1p2, const T& l2p1, const T& l2p2)
{
	T cp;
	GetCrossPoint(l1p1, l1p2, l2p1, l2p2, cp);
	return IsPointBelongsToSegment(l1p1, l1p2, cp) && IsPointBelongsToSegment(l2p1, l2p2, cp);
}

/// Rotates vector clockwise
template<typename T>
inline T RotateCW(const T& v)
{
	return T(v.y, -v.x);
}

/// Rotate vector counterclockwise
template<typename T>
inline T RotateCCW(const T& v)
{
	return T(-v.y, v.x);
}