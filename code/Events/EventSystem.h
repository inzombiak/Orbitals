#pragma once
#include "IEventData.h"
#include "../ISystem.h"

#include <functional>
#include <list>
#include <map>
#include <queue>

typedef std::function<void(IEventData*)> EventDelegate;

class EventSystem : public ISystem
{
public:
	
	bool AddEventListener(EventDefs::EventType type, EventDelegate callBack);
	bool RemoveEventListener(EventDefs::EventType type, EventDelegate callBack);
	void QueueEvent(EventDefs::EventType type, IEventData* data, bool isSynchronous);
	void Update(float dt);

	void Destroy();

	std::string GetName()
	{
		return SYSTEM_NAME;
	}
	bool Init() { return true; };

	static EventSystem* GetInstance()
	{
		if (!m_instance)
			m_instance = new EventSystem();

		return m_instance;
	}

private:

	typedef std::list<EventDelegate> EventDelegateList;
	typedef std::map<EventDefs::EventType, EventDelegateList> CallbackMap;
	typedef std::queue<IEventData*> EventQueue;
	typedef std::map<EventDefs::EventType, bool> SynchronousEventMap;

	static SynchronousEventMap SetSynchronousEvents();

	EventSystem() {};
	static const std::string SYSTEM_NAME;
	static EventSystem* m_instance;
	static CallbackMap m_eventCallbacks;
	static EventQueue m_eventQueue;
	static const SynchronousEventMap m_isSyncEventList;
};

