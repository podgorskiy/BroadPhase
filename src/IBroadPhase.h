#pragma once
class IBroadPhase : public PerformanceMeasurementsHelper
{
public:
	virtual ~IBroadPhase(){};

	virtual void UpdateAABBList(const std::vector<IBody*>& bodiesList) = 0;
	virtual const CStyleArray<int>& GenerateOverlapList() = 0;
};
