#pragma once
#include "EventDefs.h"
namespace EventDefs
{
	enum EventType
	{
		NULL_TYPE,
		CREATE_OBJECT,
		CREATE_PHYSICS_COMPONENT,
		CREATE_RENDER_COMPONENT,
		CREATE_CONSTRAINT,
		DELETE_OBJECT,
		DELETE_PHYSICS_COMPONENT,
		DELETE_RENDER_COMPONENT,
		SELECT_OBJECT_CLICK,
		GET_OBJECT_SETTINGS,
		GET_PHYSICS_DATA,
		APPLY_FORCE,
	};
}

class IEventData
{
	typedef unsigned int EventID;
public:
	IEventData(){ m_eventType = EventDefs::EventType::NULL_TYPE; m_delete = false; m_isSynchronous = false; };
	virtual ~IEventData() {} ;

	virtual const EventDefs::EventType GetEventType() const
	{
		return m_eventType;
	}

	void SetDelete(bool flag)
	{
		m_delete = flag;
	}

	bool ShouldDelete()
	{
		return m_delete;
	}

	bool GetIsSynchronous() const
	{
		return m_isSynchronous;
	}
	void SetIsSynchrnous(bool flag)
	{
		m_isSynchronous = flag;
	}

protected:
	bool m_delete = false;
	bool m_isSynchronous = false;
	EventID m_id = 0;
	EventDefs::EventType m_eventType = EventDefs::EventType::NULL_TYPE;
};

