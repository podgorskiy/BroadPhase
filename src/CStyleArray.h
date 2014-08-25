#pragma once

template <typename T>
class _CStyleArray
{
public:
	_CStyleArray() : m_data(NULL), m_size(0){};
	_CStyleArray(std::vector<T>& vector) : m_data(&vector[0]), m_size(vector.size()){};

protected:
	T*		m_data;
	size_t  m_size;

private:
	_CStyleArray(const _CStyleArray& other) {};
	_CStyleArray& operator =(const _CStyleArray& other){};
};

template <typename T>
class ConstCStyleArray : public _CStyleArray<T>
{
public:
	ConstCStyleArray(std::vector<T>& vector) : _CStyleArray<T>(vector){};

	const size_t& size()	const	{ return m_size; };

	typedef const T* pT;
	operator const pT& ()	const	{ return m_data; };
};

template <typename T>
class CStyleArray : public _CStyleArray<T>
{
public:
	CStyleArray() : _CStyleArray<T>(){};
	CStyleArray(std::vector<T>& vector) : _CStyleArray<T>(vector){};

	size_t& size()					{ return m_size; };
	const size_t& size()	const	{ return m_size; };

	typedef T* pT;
	operator const pT& ()	const	{ return m_data; };
	void push_back(const T& value);
	const T& pop_back();
	void New(size_t size);
	void Delete();
};

template <typename T>
inline void CStyleArray<T>::push_back(const T& value)
{
	m_data[m_size++] = value;
}

template <typename T>
inline const T& CStyleArray<T>::pop_back()
{
	return m_data[--m_size];
}

template <typename T>
inline void CStyleArray<T>::New(size_t size)
{
	Delete();
	m_data = new T[size];
}

template <typename T>
inline void CStyleArray<T>::Delete()
{
	if (m_data != NULL)
	{
		delete m_data;
	}
}
