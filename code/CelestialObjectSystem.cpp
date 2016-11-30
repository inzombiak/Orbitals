#include "CelestialObjectSystem.h"
#include "Events/EDCreateObject.h"
#include "Events/EDCreateRenderComp.h"
#include "Events/EventSystem.h"

#include <functional>

CelestialObjectSystem::~CelestialObjectSystem()
{
	Clear();
}

bool CelestialObjectSystem::Init()
{
	EventSystem::GetInstance()->AddEventListener(EventDefs::CREATE_OBJECT, std::bind(&CelestialObjectSystem::CreateObject, this, std::placeholders::_1));
	return true;
}

void CelestialObjectSystem::Update(float dt)
{
	//for (unsigned int i = 0; i < m_objects.size(); ++i)
		//m_objects[i]->Update();
}

void CelestialObjectSystem::CreateObject(IEventData* data)
{
	EDCreateObject* objData = dynamic_cast<EDCreateObject*>(data); 
	if (!objData)
		return;

	bool isSync = objData->GetIsSynchronous();

	ICelestialObject* newObject = new ICelestialObject(objData->GetData()->name, m_objects.size(), CelestialObjTypes::Planet);

	objData->GetData()->renderCompData->owner = newObject;

	EDCreateRenderComp* renderEventData = new EDCreateRenderComp(objData->GetData()->renderCompData);
	EventSystem::GetInstance()->QueueEvent(EventDefs::CREATE_RENDER_COMPONENT, renderEventData, isSync);

	if (isSync)
		objData->GetData()->createdObject = newObject;

	m_objects.push_back(newObject);
}

void CelestialObjectSystem::Clear()
{
	for (unsigned int i = 0; i < m_objects.size(); ++i)
		delete m_objects[i];
	EventSystem::GetInstance()->RemoveEventListener(EventDefs::CREATE_OBJECT, std::bind(&CelestialObjectSystem::CreateObject, this, std::placeholders::_1));

	m_objects.clear();
}

const std::string CelestialObjectSystem::SYSTEM_NAME = "CelestialObjectSystem";