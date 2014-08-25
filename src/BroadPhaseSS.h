#pragma once

/// Uses Sort and sweep algorithm 
class BroadPhaseSS : public IBroadPhase
{
	class EndPoint
	{
	public:
		enum Type
		{
			Minimum,
			Maximum
		};
		enum
		{
			MAXIMUM = 0x80000000,
			TYPE_LG2 = 31
		};

		void Set(unsigned short aabbIndex, float value, Type type)
		{
			m_data = type == Minimum ? aabbIndex : aabbIndex | MAXIMUM;
			m_value = value;
		};

		int GetIndex() const
		{
			return m_data & ~MAXIMUM;
		};

		int GetType() const
		{
			return m_data >> TYPE_LG2;
		};

		int GetLookUp() const
		{
			return 2 * (m_data & ~MAXIMUM) + ((m_data & MAXIMUM) >> TYPE_LG2);
		};

		bool operator < (const EndPoint& other) const
		{
			if (m_value == other.m_value)
			{
				return GetType() < other.GetType();
			}
			else
			{
				return m_value < other.m_value;
			}
		};

		float m_value;
		unsigned long m_data;
	};

	class Event
	{
	public:
		enum Type
		{
			Remove,
			Insert
		};
		enum
		{
			INSERT = 0x80000000,
			OFFSET_LG2 = 16,
			TYPE_LG2 = 31,
			MASK0 = 0x0000FFFF,
			MASK1 = 0x7FFF0000
		};
		Event(int data) :m_data(data)
		{};
		Event(int index0, int index1, Type type)
		{
			m_data = index0 | (index1 << OFFSET_LG2) | (type == Insert ? INSERT : 0);
		};
		int GetType() const
		{
			return m_data >> TYPE_LG2;
		}
		int GetKey() const
		{
			return m_data & ~INSERT;
		}
		int GetIndex0() const
		{
			return m_data & MASK0;
		}
		int GetIndex1() const
		{
			return (m_data & MASK1) >> OFFSET_LG2;
		}
		operator int() const
		{
			return m_data;
		};
		unsigned int m_data;
	};


public:
	BroadPhaseSS(int bodiesCount);

	virtual void UpdateAABBList(const std::vector<IBody*>& bodiesList);
	virtual const CStyleArray<int>& GenerateOverlapList();

private:
	void UpdateEndPoints();
	void UpdateIntervals(const CStyleArray<EndPoint>& endpoints, const CStyleArray<int>& lookUp);
	bool Overlapping(int i0, int i1, const int* lookUp, const EndPoint* endpoints);

	std::vector<aabb2df> m_aabbList;
	std::vector<EndPoint> m_endPointsX;
	std::vector<EndPoint> m_endPointsY;
	std::vector<int> m_lookUpX;
	std::vector<int> m_lookUpY;
	CStyleArray<int> m_events;
	unsigned int m_eventsMaxCount;

	std::map<int, short> m_ovelapsMap;

	CStyleArray<std::map<int, short>::iterator> m_iteratorList;
	CStyleArray<int> m_overlapingList;
};

