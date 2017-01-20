#ifndef PHYSICS_COMPONENT_H
#define PHYSICS_COMPONENT_H

#include "IRigidBody.h"
#include "../Objects/IObjectComponent.h"

class PhysicsComponent : public IObjectComponent
{
public:
	PhysicsComponent(unsigned int id)
	{
		m_id = id;
	}

	void Update(float dt) override;

	void SetBody(IRigidBody* body);
	const char* GetName() override
	{
		return COMPONENT_NAME;
	}

private:
	const static char* COMPONENT_NAME;

	IRigidBody* m_rigidBody;
};

#endif