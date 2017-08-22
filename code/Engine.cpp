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
	{
		ptr->PreDraw(m_drawOptions);
		ptr->Draw(m_drawOptions);
	}

}

//TODO: BAD. Use flag based initialization
void Engine::InitSim()
{
	m_engineState = Initializing;

	m_drawOptions = Orbitals::DrawingType::PHYSICS | Orbitals::DrawingType::COMPONENTS | Orbitals::DrawingType::SHADOW_MAP;

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
	boxData.rotation = glm::vec3(0, 0, 0);
	boxData.scale = glm::vec3(1, 1, 1);
	boxData.color = glm::vec3(0.f, 1.f, 0.f);
	boxData.extents = glm::vec3(60, 2, 60);
	boxData.position = glm::vec3(0, 0, 0);
	boxED = ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Box, static_cast<ObjectCreators::IDefaultShapeData*>(&boxData));
	boxED->GetData()->rigidBodyData->rbci.mass = 0;
	boxED->GetData()->rigidBodyData->rbci.enableGravity = false;
	EventSystem::GetInstance()->QueueEvent(boxED, true);
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
	
	std::srand(time(NULL));
	boxData.extents = glm::vec3(2, 2, 2);
	//boxData.rotation = glm::vec3((std::rand() % 360) * (M_PI / 180), (std::rand() % 360) * (M_PI / 180), (std::rand() % 360) * (M_PI / 180));
	//boxData.rotation = glm::vec3(0.f, M_PI_2 / 3, M_PI_2 / 3);
	boxData.position = glm::vec3(0, 3, -7);
	//boxData.position = glm::vec3(6, 5, -5);
	boxED = ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Box, static_cast<ObjectCreators::IDefaultShapeData*>(&boxData));
	boxED->GetData()->rigidBodyData->rbci.mass = 5;
	boxED->GetData()->rigidBodyData->rbci.enableGravity = false;
	EventSystem::GetInstance()->QueueEvent(boxED, true);
	std::shared_ptr<PhysicsComponent> pc;
	if (Orbitals::CheckConvertAndCastPtr<IObjectComponent, PhysicsComponent>(boxED->GetData()->createdObject->GetComponent(PhysicsComponent::COMPONENT_ID), pc))
	{
		pc->ApplyTorqueImpulse(glm::vec3(-4.f, 0.f, 0.f));
		//pc->ApplyImpulse(glm::vec3(-40.f, 0, 0.f));
	}
	/*boxData.extents = glm::vec3(20, 2, 20);
	boxData.rotation = glm::vec3(0, 0, 0);
	boxData.position = glm::vec3(0, 0, 0);
	boxED = ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Box, static_cast<ObjectCreators::IDefaultShapeData*>(&boxData));
	boxED->GetData()->rigidBodyData->rbci.enableGravity = false;
	EventSystem::GetInstance()->QueueEvent(boxED, false);*/

	//boxData.position = glm::vec3(4.2, 10, -5);
	//boxData.extents = glm::vec3(3, 3, 3);
	//boxData.extents = glm::vec3(2, 2, 2);
	boxData.position = glm::vec3(0, 14, 0);
	boxED = ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Box, static_cast<ObjectCreators::IDefaultShapeData*>(&boxData));
	boxED->GetData()->rigidBodyData->rbci.mass = 5;
	//boxED->GetData()->rigidBodyData->rbci.enableGravity = false;
	EventSystem::GetInstance()->QueueEvent(boxED, false);
	boxData.color = glm::vec3(1.f, 0.f, 0.f);
	boxData.position = glm::vec3(0, 18, 0);
	boxED = ObjectCreators::DefaultObjectFactory::Instance().CreateObject(ObjectCreators::DefaultShapeType::Box, static_cast<ObjectCreators::IDefaultShapeData*>(&boxData));
	boxED->GetData()->rigidBodyData->rbci.mass = 1;
	EventSystem::GetInstance()->QueueEvent(boxED, false);
	boxData.color = glm::vec3(1.f, 0.f, 0.f);
	boxData.position = glm::vec3(0, 22, 0);
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

void Engine::ClearRender()
{
	auto it = m_systemMap.find(Orbitals::SystemPriority::SRendering);
	if (it == m_systemMap.end())
		return;

	std::shared_ptr<RenderingSystem> ptr;
	if (CheckConvertAndCastPtr<ISystem, RenderingSystem>(it->second, ptr))
		ptr->ClearRender();
}

void Engine::Step(double elapsedTime)
{
	if (m_engineState != Running)
		return;
	//printf("TIME: %lf \n", elapsedTime);
	int frames = 0;
	m_accumulator += elapsedTime;
	//while (m_accumulator >= DELTA_T)
	//{
		ClearRender();
		auto it = m_systemMap.begin();
		while (it != m_systemMap.end())
		{
			it->second->Update(elapsedTime);
			it++;
		}
		m_accumulator -= DELTA_T;
		ORB_DBG::Instance().DisplayFPS(elapsedTime);
		frames++;
	//}
//	printf("FRAMES %i \n", frames);

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


