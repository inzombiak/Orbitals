#ifndef PHYSICS_WORLD_H
#define PHYSICS_WORLD_H

#include <vector>

#include <glm\glm.hpp>

#include "IRigidBody.h"

class PhysDebugDrawer;
class IBroadphase;
class INarrowphase;
class IConstraintSolver;
class PhysicsWorld
{
public:

	PhysicsWorld(IBroadphase* broadphase, INarrowphase* narrowphase, IConstraintSolver* constraintSolver);

	void AddRigidBody(IRigidBody* body);
	void RemoveRigidBody(IRigidBody* body);
	void SetGravity(const glm::vec3& grav);
	void StepSimulation(float timeStep, int maxSubStep = 1, float fixedTimeStep = 1.f/60.f);
	
	void SetPhysDebugDrawer(PhysDebugDrawer* pdd);

private:
	void ApplyGravity();
	void PredictMotion(float timeStep);
	void PerformMovement(float timeStep);
	void PerformCollisionCheck(float timeStep);

	void ClearForces();
	
	void SingleSimulationStep(float fixedTimeStep);

	// Called to draw points narrowphase fails
	static void NarrowphaseErrorCalback(std::vector<glm::vec3> finalResult);
	// Stores error
	static std::vector<glm::vec3> m_narrowphaseError;
	std::vector<PhysicsDefs::CollPairContactInfo> m_narrowphaseResult;
	
	float m_fixedTimeStep = 0;
	float m_localTime = 0;

	std::vector<IRigidBody*> m_nonStaticRigidBodies;
	std::vector<IRigidBody*> m_staticRigidBodies;

	glm::vec3 m_gravity;

	PhysDebugDrawer* m_physDebugDrawer = 0;
	IBroadphase* m_broadphase = 0;
	INarrowphase* m_narrowphase = 0;
	IConstraintSolver* m_constraintSolver;

};

#endif