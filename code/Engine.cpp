#define _USE_MATH_DEFINES
#include <cmath>

#include "Engine.h"

#include "Rendering/RenderingSystem.h"

#include "Input/InputSystem.h"

#include "Objects/ICelestialObject.h"
#include "Objects/CelestialObjectSystem.h"
#include "Objects\ObjectCreatorFunctions.h"

#include "Physics/PhysicsSystem.h"

#include "Events/EDCreateObject.h"
#include "Events/EventSystem.h"

#include "Utilities\Debug.h"

Engine::Engine()
{

}

Engine::~Engine()
{
	auto it = m_systemMap.begin();

	// Iterate over the map using iterator
	while (it != m_systemMap.end())
	{
		it->second->Destroy();
		it++;
	}

	m_systemMap.clear();
}

void Engine::Init()
{
	InitSim();
}

void Engine::Draw()
{
	auto it = m_systemMap.find(Orbitals::SystemPriority::SRendering);
	if (it == m_systemMap.end())
		return;

	std::shared_ptr<RenderingSystem> ptr;
	if (CheckConvertAndCastPtr<ISystem, RenderingSystem>(it->second, ptr))
		ptr->Draw();
}

//TODO: BAD. Use flag based initialization
void Engine::InitSim()
{
	m_engineState = Initializing;

	Functor<EDCreateObject*, TYPELIST_1(ObjectCreators::IDefaultShapeData*)> SphereDataCreator(&ObjectCreators::CreateSphereEventData);
	Functor<EDCreateObject*, TYPELIST_1(ObjectCreators::IDefaultShapeData*)> BoxDataCreator(&ObjectCreators::CreateBoxEventData);

	ObjectCreators::DefaultObjectFactory::Instance().Register(ObjectCreators::DefaultShapeType::Box, BoxDataCreator);
	ObjectCreators::DefaultObjectFactory::Instance().Register(ObjectCreators::DefaultShapeType::Sphere, SphereDataCreator);

	RenderingSystem* sys = new RenderingSystem();
	if (sys)
		sys->Init();
	m_systemMap[sys->GetPriority()] = std::shared_ptr<RenderingSystem>(sys);

	EventSystem* sys2 = EventSystem::GetInstance();
	if (sys2)
		sys2->Init();
	m_systemMap[sys2->GetPriority()] = std::shared_ptr<EventSystem>(sys2);

	CelestialObjectSystem* sys3 = new CelestialObjectSystem();
	if (sys3)
		sys3->Init();
	m_systemMap[sys3->GetPriority()] = std::shared_ptr<CelestialObjectSystem>(sys3);

	InputSystem* sys4 = InputSystem::GetInstance();
	if (sys4)
		sys4->Init();
	m_systemMap[sys4->GetPriority()] = std::shared_ptr<InputSystem>(sys4);

	PhysicsSystem* sys5 = new PhysicsSystem();
	if (sys5)
		sys5->Init();
	m_systemMap[sys5->GetPriority()] = std::shared_ptr<PhysicsSystem>(sys5);

	sys5->SetPhysDebugDrawer(sys->GetPhysDebugDrawer());

	Test();
	m_engineState = Running;
}

void Engine::Reset()
{
	InitSim();
}

void Engine::Test()
{
	ObjectCreators::SphereShapeData sphereData;
	sphereData.radius = 2;
	sphereData.position = glm::vec3(0, 0, 0);
	sphereData.rotation = glm::vec3(0, 0, 0);
	sphereData.scale = glm::vec3(1, 1, 1);
	sphereData.color = glm::vec3(0.f, 0, 1.f);

	EDCreateObject* boxED;
	ObjectCreators::BoxShapeData boxData;
	boxData.extents = glm::vec3(20, 3, 20);
	boxData.position = glm::vec3(0, 0, 0);
	boxData.rotation = glm::vec3(0, 0, 0);
	boxData.scale = glm::vec3(1, 1, 1);
	boxData.color = glm::vec3(0.f, 1.f, 0.f);


	boxED = ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Box, static_cast<ObjectCreators::IDefaultShapeData*>(&boxData));
	boxED->GetData()->rigidBodyData->rbci.enableGravity = false;
	EventSystem::GetInstance()->QueueEvent(boxED, false);
	
	/*
	EventSystem::GetInstance()->QueueEvent(ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Sphere, static_cast<ObjectCreators::IDefaultShapeData*>(&sphereData)), false);
	sphereData.position = glm::vec3(3, 8, -5);
	EventSystem::GetInstance()->QueueEvent(ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Sphere, static_cast<ObjectCreators::IDefaultShapeData*>(&sphereData)), false);
	sphereData.position = glm::vec3(5, 32, -5);
	EventSystem::GetInstance()->QueueEvent(ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Sphere, static_cast<ObjectCreators::IDefaultShapeData*>(&sphereData)), false);
	sphereData.position = glm::vec3(3, 24, -5);
	EventSystem::GetInstance()->QueueEvent(ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Sphere, static_cast<ObjectCreators::IDefaultShapeData*>(&sphereData)), false);
	sphereData.position = glm::vec3(3, 74, -5);
	EventSystem::GetInstance()->QueueEvent(ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Sphere, static_cast<ObjectCreators::IDefaultShapeData*>(&sphereData)), false);
	sphereData.position = glm::vec3(3, 64, -5);
	EventSystem::GetInstance()->QueueEvent(ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Sphere, static_cast<ObjectCreators::IDefaultShapeData*>(&sphereData)), false);
	sphereData.position = glm::vec3(3, 44, -5);
	EventSystem::GetInstance()->QueueEvent(ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Sphere, static_cast<ObjectCreators::IDefaultShapeData*>(&sphereData)), false);
	sphereData.position = glm::vec3(3, 54, -5);
	EventSystem::GetInstance()->QueueEvent(ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Sphere, static_cast<ObjectCreators::IDefaultShapeData*>(&sphereData)), false);
	boxData.position = glm::vec3(3, 94, -5);
	EventSystem::GetInstance()->QueueEvent(ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Box, static_cast<ObjectCreators::IDefaultShapeData*>(&boxData)), false);
	*/
	
	
	boxData.extents = glm::vec3(2, 2, 2);
	//boxData.rotation = glm::vec3(0, -M_PI_2 /3, 0);
	boxData.position = glm::vec3(0, 14, 0);
	//boxData.position = glm::vec3(6, 5, -5);
	boxED = ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Box, static_cast<ObjectCreators::IDefaultShapeData*>(&boxData));
	boxED->GetData()->rigidBodyData->rbci.mass = 1;
	//boxED->GetData()->rigidBodyData->rbci.enableGravity = false;
	EventSystem::GetInstance()->QueueEvent(boxED, true);
	std::shared_ptr<PhysicsComponent> pc;
	if (Orbitals::CheckConvertAndCastPtr<IObjectComponent, PhysicsComponent>(boxED->GetData()->createdObject->GetComponent(PhysicsComponent::COMPONENT_ID), pc))
	{
//		pc->ApplyTorqueImpulse(glm::vec3(2.f, 2.f, 0.f));
		//pc->ApplyImpulse(glm::vec3(2.f, 0.f, 0.f));
	}
	//boxData.position = glm::vec3(4.2, 10, -5);
	//boxData.extents = glm::vec3(3, 3, 3);
	/*boxData.position = glm::vec3(0, 10, 0);
	boxED = ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Box, static_cast<ObjectCreators::IDefaultShapeData*>(&boxData));
	boxED->GetData()->rigidBodyData->rbci.mass = 1;
	EventSystem::GetInstance()->QueueEvent(boxED, false);*/
	boxData.color = glm::vec3(1.f, 0.f, 0.f);
	boxData.position = glm::vec3(0, 30, 0);
	boxED = ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Box, static_cast<ObjectCreators::IDefaultShapeData*>(&boxData));
	boxED->GetData()->rigidBodyData->rbci.mass = 1;
	EventSystem::GetInstance()->QueueEvent(boxED, false);

	/*boxData.position = glm::vec3(6, 20, -5);
	boxED = ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Box, static_cast<ObjectCreators::IDefaultShapeData*>(&boxData));
	boxED->GetData()->rigidBodyData->rbci.mass = 20;
	EventSystem::GetInstance()->QueueEvent(boxED, false);
	boxData.position = glm::vec3(6, 30, -5);
	boxED = ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Box, static_cast<ObjectCreators::IDefaultShapeData*>(&boxData));
	boxED->GetData()->rigidBodyData->rbci.mass = 5;
	EventSystem::GetInstance()->QueueEvent(boxED, false);
	*/
	
	//boxData.position = glm::vec3(0, 0, 0);
	//boxData.extents = glm::vec3(20, 3, 20);
	//boxED = ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Box, static_cast<ObjectCreators::IDefaultShapeData*>(&boxData));
	//boxED->GetData()->rigidBodyData->rbci.enableGravity = false;
	//EventSystem::GetInstance()->QueueEvent(boxED, false); 

}

void Engine::Step(double dt)
{
	if (m_engineState != Running)
		return;

	auto it = m_systemMap.begin();

	while (it != m_systemMap.end())
	{
		it->second->Update(dt);
		it++;
	}

	ORB_DBG::Instance().DisplayFPS(dt);
	Draw();
}

void Engine::SetKey(Input::InputKey key, bool value)
{
	std::shared_ptr<InputSystem> ptr;
	if(FindSystem(SystemPriority::SInput, ptr))
		ptr->SetKeyDown(key, value);
}

void Engine::SetMousePosition(int x, int y)
{
	glm::vec2 pos;
	pos.x = -x;
	pos.y = -y;
	std::shared_ptr<InputSystem> ptr;
	if (FindSystem(SystemPriority::SInput, ptr))
		ptr->SetMousePosition(pos);
}
void Engine::SetMouseWheelRotation(float rot)
{
	std::shared_ptr<InputSystem> ptr;
	if (FindSystem(SystemPriority::SInput, ptr))
		ptr->SetMouseWheelRotation(rot);
}
void Engine::SetLeftBtn(bool value)
{
	std::shared_ptr<InputSystem> ptr;
	if (FindSystem(SystemPriority::SInput, ptr))
		ptr->SetMouseLeftButton(value);
}
void Engine::SetRightBtn(bool value)
{
	std::shared_ptr<InputSystem> ptr;
	if (FindSystem(SystemPriority::SInput, ptr))
		ptr->SetMouseRightButton(value);
}
void Engine::SetMiddleBtn(bool value)
{
	std::shared_ptr<InputSystem> ptr;
	if (FindSystem(SystemPriority::SInput, ptr))
		ptr->SetMouseWheelButton(value);
}


