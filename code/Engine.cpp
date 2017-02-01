#define _USE_MATH_DEFINES
#include <cmath>

#include "Engine.h"

#include "Rendering/RenderingSystem.h"
#include "Rendering/GLRenderHelpers.h"

#include "Input/InputSystem.h"

#include "Objects/ICelestialObject.h"
#include "Objects/CelestialObjectSystem.h"

#include "Physics/PhysicsDefs.h"
#include "Physics/PhysicsSystem.h"

#include "Events/EDCreateObject.h"
#include "Events/EventSystem.h"

#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

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
		glm::vec3 position	= glm::vec3(6, 16, -5);
		glm::vec3 rotation	= glm::vec3(0, 0, 0);
		glm::vec3 scale		= glm::vec3(1, 1, 1);
		double radius = 3;
		CreateSphere(glm::vec3(0, 0, 0), radius, vertices, normals, cleanVert, cleanIndices);

		IndexVBO(vertices, normals,
			ballData->renderCompData->indicies, ballData->renderCompData->vertices, ballData->renderCompData->normals);
		ballData->renderCompData->verticesClean = cleanVert;
		ballData->renderCompData->indiciesClean = cleanIndices;
		
		PhysicsDefs::SphereCreationData* scd = new PhysicsDefs::SphereCreationData();
		scd->sphereRadius = radius;

		scd->rbci.mass				= 10.f;
		scd->rbci.linearDamping		= 0.f;
		scd->rbci.angularDamping	= 0.f;
		scd->rbci.friction			= 0.f;
		scd->rbci.rollingFriction	= 0.f;
		scd->rbci.resititution		= 0.f;

		glm::mat4 translationMat	= glm::translate(position);
		glm::mat4 scalingMat		= glm::scale(scale);
		glm::mat4 rotationMat		= glm::toMat4(glm::quat(rotation));
		scd->rbci.transform			= translationMat * rotationMat * scalingMat;
		scd->rbci.localInertia = glm::vec3(0, 0, 0); //TODO: INCORRECT, add calculator funciton;

		ballData->rigidBodyData = scd;

		//ballData->renderCompData->normals = normals;
		auto eventData = new EDCreateObject(ballData);
		EventSystem::GetInstance()->QueueEvent(eventData, false);
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
		float boxWidth = 4, boxLength = 4, boxHeight = 4;
		glm::vec3 position = glm::vec3(6, 0, -5);
		glm::vec3 rotation = glm::vec3(0, 0, 0);
		glm::vec3 scale = glm::vec3(1, 1, 1);
		CreateBox(glm::vec3(0, 0, 0), boxWidth, boxHeight, boxLength, vertices, normals, cleanVert, cleanIndices);
		IndexVBO(vertices, normals,
			boxData->renderCompData->indicies, boxData->renderCompData->vertices, boxData->renderCompData->normals);

		PhysicsDefs::BoxCreationData* bcd = new PhysicsDefs::BoxCreationData();
		bcd->boxDimensions = glm::vec3(boxWidth, boxHeight, boxLength);

		bcd->rbci.mass = 0.f;
		bcd->rbci.linearDamping = 0.f;
		bcd->rbci.angularDamping = 0.f;
		bcd->rbci.friction = 0.f;
		bcd->rbci.rollingFriction = 0.f;
		bcd->rbci.resititution = 0.f;

		glm::mat4 translationMat = glm::translate(position);
		glm::mat4 scalingMat = glm::scale(scale);
		glm::mat4 rotationMat = glm::toMat4(glm::quat(rotation));
		bcd->rbci.transform = translationMat * rotationMat * scalingMat;
		bcd->rbci.localInertia = glm::vec3(0, 0, 0); //TODO: INCORRECT, add calculator funciton;

		boxData->rigidBodyData = bcd;

		boxData->renderCompData->verticesClean = cleanVert;
		boxData->renderCompData->indiciesClean = cleanIndices;
		auto eventData = new EDCreateObject(boxData);
		EventSystem::GetInstance()->QueueEvent(eventData, false);
	}
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


