#pragma once
#include "IEventData.h"

#include <glm\glm.hpp>
#include <vector>

class EDCreateRenderComp :
	public IEventData
{
public:
	EDCreateRenderComp(RenderCompCreationData* data)
	{ 
		m_renderCompCreationData = data;
		m_eventType = EventDefs::CREATE_RENDER_COMPONENT;
	};

	RenderCompCreationData* GetData()
	{
		return m_renderCompCreationData;
	}

	~EDCreateRenderComp()
	{
		delete m_renderCompCreationData;
		m_renderCompCreationData = 0;
	}

private:
	RenderCompCreationData* m_renderCompCreationData;
};

