#define _USE_MATH_DEFINES
#include <cmath>

#include "Engine.h"
#include "Rendering/RenderingSystem.h"
#include "InputSystem.h"
#include "CelestialObjectSystem.h"
#include "Utilities/GLRenderHelpers.h"
#include "Utilities/PhysicsDefs.h"
#include "Events/EventSystem.h"
#include "ICelestialObject.h"
#include "Events/EDCreateObject.h"

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
	auto it = m_systemMap.find(SystemID(HashedString::hash_name(RenderingSystem::GetName().c_str())));
	if (it == m_systemMap.end())
		return;

	std::shared_ptr<RenderingSystem> ptr;
	if (CheckConvertAndCastPtr<ISystem, RenderingSystem>(it->second, ptr))
		ptr->Draw();
}

void Engine::InitSim()
{
	m_engineState = Initializing;

	RenderingSystem* sys = new RenderingSystem();
	if (sys)
		sys->Init();
	m_systemMap[SystemID(HashedString::hash_name(sys->GetName().c_str()))] = std::shared_ptr<RenderingSystem>(sys);

	EventSystem* sys2 = EventSystem::GetInstance();
	if (sys2)
		sys2->Init();
	m_systemMap[SystemID(HashedString::hash_name(sys2->GetName().c_str()))] = std::shared_ptr<EventSystem>(sys2);

	CelestialObjectSystem* sys3 = new CelestialObjectSystem();
	if (sys3)
		sys3->Init();
	m_systemMap[SystemID(HashedString::hash_name(sys3->GetName().c_str()))] = std::shared_ptr<CelestialObjectSystem>(sys3);

	InputSystem* sys4 = InputSystem::GetInstance();
	if (sys4)
		sys4->Init();
	m_systemMap[SystemID(HashedString::hash_name(sys4->GetName().c_str()))] = std::shared_ptr<InputSystem>(sys4);

	Test();
	m_engineState = Running;
}

void Engine::Reset()
{
	InitSim();
}

void Engine::Test()
{
	{
		ObjectCreationData* ballData = new ObjectCreationData();
		ballData->name = "ball";
		ballData->renderCompData->color.push_back(glm::vec3(0.f, 0, 1.f));
		ballData->renderCompData->drawType = GL_TRIANGLES;


		std::vector<glm::vec3> vertices;
		std::vector<unsigned short> faceIndices;
		std::vector<unsigned short> cleanIndices;
		std::vector<glm::vec3> normals;
		std::vector<glm::vec3> cleanVert;

		CreateSphere(glm::vec3(0, 0, -5), 3, vertices, normals, cleanVert, cleanIndices);

		IndexVBO(vertices, normals,
			ballData->renderCompData->indicies, ballData->renderCompData->vertices, ballData->renderCompData->normals);
		ballData->renderCompData->verticesClean = cleanVert;
		ballData->renderCompData->indiciesClean = cleanIndices;
		
		//ballData->renderCompData->normals = normals;
		auto eventData = new EDCreateObject(ballData);
		EventSystem::GetInstance()->QueueEvent(EventDefs::CREATE_OBJECT, eventData, false);
	}

	{
		ObjectCreationData* boxData = new ObjectCreationData();
		boxData->name = "box";
		boxData->renderCompData->color.push_back(glm::vec3(0.f, 1.f, 0));
		boxData->renderCompData->drawType = GL_TRIANGLES;


		std::vector<glm::vec3> vertices;
		std::vector<unsigned short> faceIndices;
		std::vector<unsigned short> cleanIndices;
		std::vector<glm::vec3> normals;
		std::vector<glm::vec3> cleanVert;

		CreateBox(glm::vec3(6, 0, -5), 4, 4, 4, vertices, normals, cleanVert, cleanIndices);
		IndexVBO(vertices, normals,
			boxData->renderCompData->indicies, boxData->renderCompData->vertices, boxData->renderCompData->normals);
		boxData->renderCompData->verticesClean = cleanVert;
		boxData->renderCompData->indiciesClean = cleanIndices;
		auto eventData = new EDCreateObject(boxData);
		EventSystem::GetInstance()->QueueEvent(EventDefs::CREATE_OBJECT, eventData, false);
	}
}

void Engine::Step(double dt)
{
	if (m_engineState != Running)
		return;

	auto it = m_systemMap.begin();

	// Iterate over the map using iterator
	while (it != m_systemMap.end())
	{
		it->second->Update(dt);
		it++;
	}
	Draw();
}

void Engine::SetKey(Input::InputKey key, bool value)
{
	std::shared_ptr<InputSystem> ptr;
	if(FindSystem(SystemID(HashedString::hash_name(InputSystem::GetName().c_str())), ptr))
		ptr->SetKeyDown(key, value);
}

void Engine::SetMousePosition(int x, int y)
{
	glm::vec2 pos;
	pos.x = -x;
	pos.y = -y;
	std::shared_ptr<InputSystem> ptr;
	if (FindSystem(SystemID(HashedString::hash_name(InputSystem::GetName().c_str())), ptr))
		ptr->SetMousePosition(pos);
}
void Engine::SetMouseWheelRotation(float rot)
{
	std::shared_ptr<InputSystem> ptr;
	if (FindSystem(SystemID(HashedString::hash_name(InputSystem::GetName().c_str())), ptr))
		ptr->SetMouseWheelRotation(rot);
}
void Engine::SetLeftBtn(bool value)
{
	std::shared_ptr<InputSystem> ptr;
	if (FindSystem(SystemID(HashedString::hash_name(InputSystem::GetName().c_str())), ptr))
		ptr->SetMouseLeftButton(value);
}
void Engine::SetRightBtn(bool value)
{
	std::shared_ptr<InputSystem> ptr;
	if (FindSystem(SystemID(HashedString::hash_name(InputSystem::GetName().c_str())), ptr))
		ptr->SetMouseRightButton(value);
}
void Engine::SetMiddleBtn(bool value)
{
	std::shared_ptr<InputSystem> ptr;
	if (FindSystem(SystemID(HashedString::hash_name(InputSystem::GetName().c_str())), ptr))
		ptr->SetMouseWheelButton(value);
}


