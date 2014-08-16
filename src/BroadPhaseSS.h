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
		void Set(unsigned short aabbIndex, float value, Type type)
		{
			m_aabbIndex = aabbIndex;
			m_value = value;
			m_type = type;
		};

		int GetIndex() const
		{
			return m_aabbIndex;
		};

		Type GetType() const
		{
			return m_type;
		};

		int GetLookUp() const
		{
			return 2 * m_aabbIndex + (m_type == Maximum ? 1 : 0);
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

		float	m_value;
		Type	m_type;
		int		m_aabbIndex;
	};

	class Event
	{
	public:
		enum Type
		{
			Remove,
			Insert
		};
		
		Event(int data)
		{};
		Event(int index0, int index1, Type type)
		{
			m_index0 = index0;
			m_index1 = index1;
			m_type = type;
		};
		Type GetType() const
		{
			return m_type;
		}
		int GetIndex0() const
		{
			return m_index0;
		}
		int GetIndex1() const
		{
			return m_index1;
		}
		
		Type m_type;
		int m_index0;
		int m_index1;
	};

public:
	BroadPhaseSS(int bodiesCount);

	virtual void UpdateAABBList(const std::vector<IBody*> bodiesList);
	virtual const std::vector<std::pair<int, int> >& GenerateOverlapList();

private:
	void UpdateEndPoints();
	void UpdateIntervals(std::vector<EndPoint>& endpoints, std::vector<int>& lookUp);
	bool Overlapping(int i0, int i1, const std::vector<int>& lookUp, const std::vector<EndPoint>& endpoints);

	std::vector<aabb2df> m_aabbList;
	std::vector<EndPoint> m_endPointsX;
	std::vector<EndPoint> m_endPointsY;
	std::vector<int> m_lookUpX;
	std::vector<int> m_lookUpY;
	std::vector<Event> m_events;
	std::set<std::pair<int, int> > m_ovelapsSet;
	std::vector<std::pair<int, int> > m_overlapingList;
};

