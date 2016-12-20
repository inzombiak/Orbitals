#ifndef PHYSICS_WORLD_H
#define PHYSICS_WORLD_H

#include <vector>

#include <glm\glm.hpp>

#include "IRigidBody.h"

class PhysicsWorld
{
public:

	void AddRigidBody(IRigidBody* body);
	void SetGravity(const glm::vec3& grav);
	void StepSimulation(float timeStep, int maxSubStep, float fixedTimeStep);
	

private:
	
	std::vector<IRigidBody*> m_nonStaticRigidBodies;
	std::vector<IRigidBody*> m_staticRigidBodies;

	glm::vec3 m_gravity;
};

#endif