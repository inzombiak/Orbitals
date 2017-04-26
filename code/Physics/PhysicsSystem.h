#ifndef PHYSICS_SYSTEM_H
#define PHYSICS_SYSTEM_H

#include "../ISystem.h"

#include "PhysicsComponent.h"

class IEventData;
class PhysDebugDrawer;
class PhysicsWorld;
class INarrowphase;
class IBroadphase;
class PhysicsSystem : public ISystem
{

public:
	~PhysicsSystem();
	//Initalize render manager
	bool Init() override;
	void Destroy() override
	{
		Clear();
	}
	void Clear();
	void Update(float dt) override;

	//Create a render component
	void CreatePhysicsComponent(IEventData* eventData);

	void SetPhysDebugDrawer(PhysDebugDrawer* pdd);

	SystemPriority GetPriority()
	{
		return Orbitals::SystemPriority::SPhysics;
	}

	static std::string GetName()
	{
		return SYSTEM_NAME;
	};

private:
	void ClearPhysWorld();

	std::vector<PhysicsComponent>	m_physComponents;
	std::vector<ICollisionShape*>	m_collisionShapes;
	std::vector<IRigidBody*>		m_rigidBodies;
	IBroadphase*					m_broadphase = 0;
	INarrowphase*					m_narrowphase = 0;
	PhysicsWorld*					m_physWorld = 0;



	static const std::string SYSTEM_NAME;
};


#endif