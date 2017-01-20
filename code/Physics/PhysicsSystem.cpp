#include "PhysicsSystem.h"
#include "../Events/EDCreatePhysComp.h"
#include "../Events/EventSystem.h"

#include "BoxShape.h"
#include "SphereShape.h"

//TODO: Collision Shape reuse
//TODO: Collision Shape factory

PhysicsSystem::~PhysicsSystem()
{
	Clear();
}

bool PhysicsSystem::Init()
{
	m_physWorld.SetGravity(glm::vec3(0, -9.8f, 0));
	EventSystem::GetInstance()->AddEventListener(EventDefs::CREATE_PHYSICS_COMPONENT, std::bind(&PhysicsSystem::CreatePhysicsComponent, this, std::placeholders::_1));
	return true;
}

void PhysicsSystem::Clear()
{
	m_physComponents.clear();

	for (unsigned int i = 0; i < m_collisionShapes.size(); ++i)
	{
		if (!m_collisionShapes[i])
			continue;

		delete m_collisionShapes[i];
		m_collisionShapes[i] = 0;
	}

	for (unsigned int i = 0; i < m_rigidBodies.size(); ++i)
	{
		if (!m_rigidBodies[i])
			continue;
		m_physWorld.RemoveRigidBody(m_rigidBodies[i]);
		delete m_rigidBodies[i];
		m_rigidBodies[i] = 0;
	}

};

void PhysicsSystem::Update(float dt)
{
	m_physWorld.StepSimulation(dt);

	for (unsigned int i = 0; i < m_physComponents.size(); ++i)
		m_physComponents[i].Update(dt);
}

//TODO: bad and unclean, maybe add factory. Need to fix how and what data is transferred 
void PhysicsSystem::CreatePhysicsComponent(IEventData* eventData)
{
	EDCreatePhysComp* physCompED = dynamic_cast<EDCreatePhysComp*>(eventData);
	if (!physCompED)
		return;

	PhysicsComponent newComponent(m_physComponents.size());
	ICollisionShape* newShape;
	IRigidBody* rb;
	if (physCompED->GetData()->GetShape() == PhysicsDefs::CollisionShapeType::Box)
	{
		PhysicsDefs::BoxCreationData* bcd = dynamic_cast<PhysicsDefs::BoxCreationData*>(physCompED->GetData());
		newShape = new BoxShape(bcd->boxDimensions);
		bcd->rbci.collisionShape = newShape;
		rb = new IRigidBody(bcd->rbci);
	}
	else
	{
		PhysicsDefs::SphereCreationData* scd = dynamic_cast<PhysicsDefs::SphereCreationData*>(physCompED->GetData());
		newShape = new SphereShape(scd->sphereRadius);
		scd->rbci.collisionShape = newShape;
		rb = new IRigidBody(scd->rbci);
	}

	m_collisionShapes.push_back(newShape);
	m_rigidBodies.push_back(rb);

	newComponent.SetBody(rb);
	newComponent.SetOwner(physCompED->GetData()->owner);
	m_physComponents.push_back(newComponent);
	m_physWorld.AddRigidBody(rb);
	eventData->SetDelete(true);
}

const std::string PhysicsSystem::SYSTEM_NAME = "PhysicsSystem";