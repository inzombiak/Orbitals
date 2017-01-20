#ifndef PHYSICS_SYSTEM_H
#define PHYSICS_SYSTEM_H

#include "../ISystem.h"

#include "PhysicsWorld.h"
#include "PhysicsComponent.h"

class IEventData;
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

	SystemPriority GetPriority()
	{
		return Orbitals::SystemPriority::SPhysics;
	}

	static std::string GetName()
	{
		return SYSTEM_NAME;
	};

private:
	std::vector<PhysicsComponent>	m_physComponents;
	std::vector<ICollisionShape*>	m_collisionShapes;
	std::vector<IRigidBody*>		m_rigidBodies;
	PhysicsWorld					m_physWorld;

	static const std::string SYSTEM_NAME;
};


#endif