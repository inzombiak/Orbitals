#include "EventSystem.h"

bool EventSystem::AddEventListener(EventDefs::EventType type, EventDelegate callBack)
{
	auto callbackIt = m_eventCallbacks.find(type);
	if (callbackIt == m_eventCallbacks.end())
		m_eventCallbacks[type] = EventDelegateList();

	auto& callbackList = m_eventCallbacks[type];
	for (auto it = callbackList.begin(); it != callbackList.end(); ++it)
	{
		if (it->target<EventDelegate>() == callBack.target<EventDelegate>())
			return false;
	}

	callbackList.push_back(callBack);

	return true;
}

bool EventSystem::RemoveEventListener(EventDefs::EventType type, EventDelegate callBack)
{
	auto callbackIt = m_eventCallbacks.find(type);
	if (callbackIt == m_eventCallbacks.end())
		return false;

	auto& callbackList = callbackIt->second;
	for (auto it = callbackList.begin(); it != callbackList.end(); ++it)
	{
		if (it->target<EventDelegate>() == callBack.target<EventDelegate>())
		{
			callbackList.erase(it);
			return true;
		}
	}

	return false;
}

void EventSystem::QueueEvent(EventDefs::EventType type, IEventData* data, bool isSynchronous)
{
	if (!m_isSyncEventList.at(type) && !isSynchronous)
	{
		m_eventQueue.push(data);

		return;
	}
		
	auto callBackListIt = m_eventCallbacks.find(type);
	if (callBackListIt == m_eventCallbacks.end())
	{
		delete data;
		return;
	}

	auto& callBackList = callBackListIt->second;

	for (auto it = callBackList.begin(); it != callBackList.end(); ++it)
		(*it)(data);

	data->SetDelete(true);
	m_eventQueue.push(data);
}

void EventSystem::Update(float dt)
{
	while (!m_eventQueue.empty())
	{
		IEventData* eData = m_eventQueue.front();
		EventDefs::EventType eType = eData->GetEventType();

		auto callBackListIt = m_eventCallbacks.find(eType);
		if (callBackListIt == m_eventCallbacks.end() || eData->ShouldDelete())
		{
				delete eData;
				m_eventQueue.pop();
				continue;
		}

		auto& callBackList = callBackListIt->second;

		for (auto it = callBackList.begin(); it != callBackList.end(); ++it)
			(*it)(eData);

		delete eData;
		m_eventQueue.pop();
	}
}

void EventSystem::Destroy()
{

	while(!m_eventQueue.empty())
	{
		IEventData* eData = m_eventQueue.front();
		delete eData;
		m_eventQueue.pop();
	}

	//if (m_instance)
	//	delete m_instance;

	//m_instance = 0;
}

EventSystem::SynchronousEventMap EventSystem::SetSynchronousEvents()
{
	EventSystem::SynchronousEventMap result;

	result[EventDefs::SELECT_OBJECT_CLICK] = true;
	result[EventDefs::GET_PHYSICS_DATA] = true;
	result[EventDefs::GET_OBJECT_SETTINGS] = true;

	result[EventDefs::NULL_TYPE] = false;
	result[EventDefs::CREATE_OBJECT] = false;
	result[EventDefs::CREATE_PHYSICS_COMPONENT] = false;
	result[EventDefs::CREATE_RENDER_COMPONENT] = false;
	result[EventDefs::CREATE_CONSTRAINT] = false;
	result[EventDefs::DELETE_OBJECT] = false;
	result[EventDefs::DELETE_PHYSICS_COMPONENT] = false;
	result[EventDefs::DELETE_RENDER_COMPONENT] = false;
	result[EventDefs::APPLY_FORCE] = false;

	return result;
}

const std::string EventSystem::SYSTEM_NAME = "EventSystem";
EventSystem* EventSystem::m_instance = 0;
EventSystem::EventQueue EventSystem::m_eventQueue;
EventSystem::CallbackMap EventSystem::m_eventCallbacks;
const EventSystem::SynchronousEventMap EventSystem::m_isSyncEventList = EventSystem::SetSynchronousEvents();