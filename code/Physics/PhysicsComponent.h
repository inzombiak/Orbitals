#ifndef PHYSICS_COMPONENT_H
#define PHYSICS_COMPONENT_H

#include "IRigidBody.h"
#include "../Objects/IObjectComponent.h"

class PhysicsComponent : public IObjectComponent
{
public:
	const static char* COMPONENT_NAME;
	const static ObjComponentID COMPONENT_ID;

	PhysicsComponent(unsigned int id)
	{
		m_id = id;
	}

	void Update(float dt) override;

	void SetBody(IRigidBody* body);

	void ApplyImpulse(const glm::vec3& impulse)
	{
		m_rigidBody->ApplyImpulse(impulse);
	}
	void ApplyTorqueImpulse(const glm::vec3& torque)
	{
		m_rigidBody->ApplyTorqueImpulse(torque);
	}
	void ApplyForce(const glm::vec3& force)
	{
		m_rigidBody->ApplyForce(force);
	}

	const char* GetName() override
	{
		return COMPONENT_NAME;
	}

	ObjComponentID GetComponentID()
	{
		return COMPONENT_ID;
	}

private:

	IRigidBody* m_rigidBody;
};

#endif