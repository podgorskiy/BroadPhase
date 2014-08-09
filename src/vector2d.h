#pragma once

template <class T>
class vector2d
{
	enum VectorSize
	{
		k_size = 2
	};
public:
	vector2d() {};
	/// Construct using coordinates.
	vector2d(T x, T y) : x(x), y(y) {};
	/// Copy constructor
	vector2d(const vector2d<T>& v) : x(v.x), y(v.y) {};
	/// Construct unit vector using angle.
	explicit vector2d(T fi) : x(cosf(fi)), y(sinf(fi)) {};
	/// Set this vector by coordinates.
	void Set(const T& x_, const T& y_) { x = x_; y = y_; };

	/// Negate this vector.
	vector2d<T> operator - () const					{ return vector2d<T>(-x, -y); };
	/// Add a vector to this vector.
	vector2d<T>& operator += (const vector2d<T>& v)	{ x += v.x; y += v.y; return *this; };
	/// Subtract a vector from this vector.
	vector2d<T>& operator -= (const vector2d<T>& v)	{ x -= v.x; y -= v.y; return *this; };
	/// Multiply this vector by a scalar.
	vector2d<T>& operator *= (const T a)			{ x *= a; y *= a;  return *this; };
	/// Divide this vector by a scalar.
	vector2d<T>& operator /= (const T a)			{ T i = (T)1.0 / a; x *= i; y *= i; return *this; };
	/// Assign operator.
	vector2d<T>& operator = (const vector2d<T>& v)	{ x = v.x; y = v.y; return *this; };
	/// Read from and indexed element.
	T operator [] (int i) const						{ return (&x)[i]; };
	/// Write to an indexed element.
	T& operator [] (int i)							{ return (&x)[i]; };
	/// Get length of the vector.
	T GetLength() const								{ return sqrt(GetLengthSQ()); };
	/// Get squared length of the vector.
	T GetLengthSQ() const							{ return x * x + y * y; };
	/// Get normalized vector.
	vector2d<T> GetNormalized()	const
	{
		T length = GetLength();
		if (length > FLT_EPSILON)
		{
			return vector2d<T>(*this) / length;
		}
		return *this;
	};
	/// Normalizes the vector.
	vector2d<T>& Normalize()
	{
		return *this = GetNormalized();
	};
	/// Returns size of this vector
	static int GetSize()
	{
		return k_size;
	}

	T x, y;
};

/// Dot product.
template <class T>
inline T operator * (const vector2d<T>& a, const vector2d<T>& b) { return a.x * b.x + a.y * b.y; }
/// Cross product.
template <class T>
inline T Cross(const vector2d<T>& a, const vector2d<T>& b) { return  a.x * b.y - a.y * b.x; }
/// Add two vectors.
template <class T>
inline vector2d<T> operator + (const vector2d<T>& a, const vector2d<T>& b) { return vector2d<T>(a.x + b.x, a.y + b.y); }
/// Subtract two vectors.
template <class T>
inline vector2d<T> operator - (const vector2d<T>& a, const vector2d<T>& b) { return vector2d<T>(a.x - b.x, a.y - b.y); }
/// Multiply a vector by a scalar.
template <class T>
inline vector2d<T> operator * (const vector2d<T>& a, const T s) { return vector2d<T>(s * a.x, s * a.y); }
/// Multiply a vector by a scalar.
template <class T>
inline vector2d<T> operator * (const T s, const vector2d<T>& a) { return vector2d<T>(s * a.x, s * a.y); }
/// Divide a vector by a scalar.
template <class T>
inline vector2d<T> operator / (const vector2d<T>& a, const T s) { return vector2d<T>(a.x / s, a.y / s); }


/// Typedef for single precision 2d vector.
typedef vector2d<float> vector2df;


#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif
