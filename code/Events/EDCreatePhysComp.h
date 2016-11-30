#pragma once
#include "IEventData.h"
#include "PhysicsDefs.h"
#include "GameDefs.h"
#include <array>

class EDCreatePhysComp :
	public IEventData
{
public:
	EDCreatePhysComp(PhysicsDefs::ICreationData* data) 
	{
		m_physCompCreationData = data;
		m_eventType = EventDefs::CREATE_PHYSICS_COMPONENT;
	};

	~EDCreatePhysComp() 
	{
		delete m_physCompCreationData; 
		m_physCompCreationData = 0;
	}

	PhysicsDefs::ICreationData* GetData()
	{
		return m_physCompCreationData;
	}

private:
	PhysicsDefs::ICreationData* m_physCompCreationData;
};

