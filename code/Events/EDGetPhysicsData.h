#pragma once
#include "IEventData.h"

#include <array>

class EDGetPhysicsData :
	public IEventData
{
public:

	EDGetPhysicsData(std::string& name)
	{
		m_name = name;
		m_eventType = EventDefs::GET_PHYSICS_DATA;
	};

	~EDGetPhysicsData()
	{

	}

	void SetData(PhysicsDefs::ICreationData* data)
	{
		m_physCompCreationData = data;
	}

	PhysicsDefs::ICreationData* GetData() const
	{
		return m_physCompCreationData;
	}

	const std::string& GetName() const
	{
		return m_name;
	}

	void SetFoundFlag(bool flag)
	{
		m_dataFound = flag;
	}

	bool WasDataFound()
	{
		return m_dataFound;
	}

private:
	bool m_dataFound;
	std::string m_name;
	PhysicsDefs::ICreationData* m_physCompCreationData;
};