#pragma once

template <template< class > class T, class A>
class aabb
{
public:
	/// Construct empty volume.
	aabb() : minp(FLT_MAX), maxp(FLT_MIN){};
	/// Construct from single point.
	aabb(T<A> p) : minp(p), maxp(p) {};
	/// Construct from min and max points.
	aabb(T<A> minp, T<A> maxp) : minp(minp), maxp(maxp) {};
	/// Copy constructor.
	aabb(const aabb<T,A>& v) : minp(v.minp), maxp(v.maxp) {};
	/// Assign operator.
	aabb<T,A>& operator = (const aabb<T,A>& v)	{ minp = v.minp; maxp = v.maxp; return *this; };
	/// Returns sizes of volume.
	T<A> GetSize() const { return maxp - minp; };
	/// Add a pont to volume expanding it.
	bool AddPoint(const T<A>& p);

	T<A> minp, maxp;
};

template <template< class > class T, class A>
inline bool aabb<T, A>::AddPoint(const T<A>& p)
{
	bool changed = false;
	for (int i = 0; i < T<A>::GetSize(); ++i)
	{
		minp[i] = p[i] < minp[i] ? changed = true, p[i] : minp[i];
		maxp[i] = p[i] > maxp[i] ? changed = true, p[i] : maxp[i];
	}
	return changed;
}

template <template< class > class T, class A>
inline aabb<T, A> operator + (const aabb<T, A>& a, const aabb<T, A>& b)
{
	aabb<T, A> box(a);
	box.AddPoint(b.maxp);
	box.AddPoint(b.minp);
	return box;
}

template <template< class > class T, class A>
inline bool Overlapping(aabb<T, A> bodyA, aabb<T, A> bodyB)
{
	for (int i = 0; i < T<A>::GetSize(); ++i)
	{
		if ((bodyA.maxp[i] < bodyB.minp[i]) || (bodyA.minp[i] > bodyB.maxp[i]))
		{
			return false;
		}
	}
	return true;
}

/// Typedef for single precision 2d aabb.
typedef aabb<vector2d, float> aabb2df;

