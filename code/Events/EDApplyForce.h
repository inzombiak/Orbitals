#pragma once
#include "IEventData.h"

class EDApplyForce :
	public IEventData
{
public:
	
	EDApplyForce(float mouseX, float mouseY, float amount)
	{
		m_mouseX = mouseX;
		m_mouseY = mouseY;
		m_amount = amount;
		m_eventType = EventDefs::APPLY_FORCE;
	}

	float GetMouseX()
	{
		return m_mouseX;
	}	

	float GetMouseY()
	{
		return m_mouseY;
	}

	float GetAmount()
	{
		return m_amount;
	}

private:
	float m_mouseX, m_mouseY, m_amount;
};

