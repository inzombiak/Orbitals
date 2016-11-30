#pragma once
#include "IEventData.h"


struct ObjectCreationData
{
	int nameID;
	std::string name;
	RenderCompCreationData* renderCompData = new RenderCompCreationData();
	ICelestialObject* createdObject = 0;
};

class EDCreateObject :
	public IEventData
{
public:
	EDCreateObject(ObjectCreationData* data) 
	{ 
		m_objCreationData = data;
		m_eventType = EventDefs::CREATE_OBJECT;
	}

	~EDCreateObject()
	{
		delete m_objCreationData;
		m_objCreationData = 0;
	}

	ObjectCreationData* GetData()
	{
		return m_objCreationData;
	}

private:
	ObjectCreationData* m_objCreationData;
};

