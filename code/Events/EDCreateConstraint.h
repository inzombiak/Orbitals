#pragma once
/*
#include "IEventData.h"
#include "PhysicsDefs.h"
#include "GameDefs.h"

class EDCreateConstraint :
	public IEventData
{
public:
	EDCreateConstraint(PhysicsDefs::ConstraintCreationData* data)
	{
		m_constCreationData = data;
		m_eventType = EventDefs::CREATE_CONSTRAINT;
	}

	~EDCreateConstraint()
	{
		delete m_constCreationData;
		m_constCreationData = 0;
	}

	PhysicsDefs::ConstraintCreationData* GetData()
	{
		return m_constCreationData;
	}

private:
	PhysicsDefs::ConstraintCreationData* m_constCreationData;
};*/