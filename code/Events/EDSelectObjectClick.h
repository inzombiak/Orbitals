#pragma once
#include "IEventData.h"
class EDSelectObjectClick :
	public IEventData
{

public:
	EDSelectObjectClick(bool retrievePhysInfo, float mouseX, float mouseY)
	{
		m_retrievePhysicsInfo = retrievePhysInfo;
		m_mouseX = mouseX;
		m_mouseY = mouseY;

		m_eventType = EventDefs::SELECT_OBJECT_CLICK;
	}
	~EDSelectObjectClick()
	{

	}

	bool RetrievePhysics()
	{
		return m_retrievePhysicsInfo;
	}

	float GetMouseX()
	{
		return m_mouseX;
	}

	float GetMouseY()
	{
		return m_mouseY;
	}

private:
	bool m_retrievePhysicsInfo;
	float m_mouseX, m_mouseY;
};

