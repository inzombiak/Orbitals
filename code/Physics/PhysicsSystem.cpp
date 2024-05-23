#include "PhysicsSystem.h"
#include "../Events/EDCreatePhysComp.h"
#include "../Events/EventSystem.h"

#include "PhysicsWorld.h"
#include "BoxShape.h"
#include "SphereShape.h"

#include "NarrowphaseSAT.h"
#include "NarrowphaseGJK_EPA.h"

#include "BroadphaseAABB.h"

#include "ConstraintSolverSeqImpulse.h"

//TODO: Collision Shape reuse
//TODO: Collision Shape factory

PhysicsSystem::~PhysicsSystem()
{
	Clear();
}

bool PhysicsSystem::Init()
{
	m_broadphase = new BroadphaseAABB();
	m_narrowphase = new NarrowphaseSAT();
	m_constraintSolver = new ConstraintSolverSeqImpulse();;
	if (m_physWorld)
	{
		ClearPhysWorld();
		delete m_physWorld;
		m_physWorld = new PhysicsWorld(m_broadphase, m_narrowphase, m_constraintSolver);
	}
	else
		m_physWorld = new PhysicsWorld(m_broadphase, m_narrowphase, m_constraintSolver);

	m_physWorld->SetGravity(glm::vec3(0, -9.8f, 0));
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

	ClearPhysWorld();

	if (m_physWorld)
		delete m_physWorld;
	if (m_broadphase)
		delete m_broadphase;
	if (m_narrowphase)
		delete m_narrowphase;
	if (m_constraintSolver)
		delete m_constraintSolver;
};

void PhysicsSystem::ClearPhysWorld()
{
	for (unsigned int i = 0; i < m_rigidBodies.size(); ++i)
	{
		if (!m_rigidBodies[i])
			continue;
		m_physWorld->RemoveRigidBody(m_rigidBodies[i]);
		delete m_rigidBodies[i];
		m_rigidBodies[i] = 0;
	}
}

void PhysicsSystem::Update(float dt)
{
	m_physWorld->StepSimulation(dt);

	for (unsigned int i = 0; i < m_physComponents.size(); ++i)
		m_physComponents[i].Update(dt);
}

//TODO: bad and unclean, maybe add factory. Need to fix how and what data is transferred 
void PhysicsSystem::CreatePhysicsComponent(IEventData* eventData)
{
	EDCreatePhysComp* physCompED =(EDCreatePhysComp*)(eventData);
	if (!physCompED)
		return;

	PhysicsComponent newComponent((unsigned int)m_physComponents.size());
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
	

	m_physComponents.push_back(newComponent);
	m_physComponents[m_physComponents.size() - 1].SetOwner(physCompED->GetData()->owner);
	m_physWorld->AddRigidBody(rb);
	eventData->SetDelete(true);
}
void PhysicsSystem::SetPhysDebugDrawer(PhysDebugDrawer* pdd)
{
	m_physWorld->SetPhysDebugDrawer(pdd);
}

const std::string PhysicsSystem::SYSTEM_NAME = "PhysicsSystem";