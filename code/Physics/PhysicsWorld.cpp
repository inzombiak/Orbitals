#include "PhysicsWorld.h"
#include "glm\gtc\matrix_transform.hpp"
#include "../Rendering/PhysDebugDrawer.h"

PhysicsWorld::PhysicsWorld()
{
	m_gravity = glm::vec3(0);
}

void PhysicsWorld::AddRigidBody(IRigidBody* body)
{
	body->SetGravity(m_gravity);
	m_nonStaticRigidBodies.push_back(body);
}

void PhysicsWorld::RemoveRigidBody(IRigidBody* body)
{
	auto it = std::find(m_nonStaticRigidBodies.begin(), m_nonStaticRigidBodies.end(), body);
	if (it != m_nonStaticRigidBodies.end())
	{
		std::swap(it, m_nonStaticRigidBodies.end() - 1);
		m_nonStaticRigidBodies.pop_back();
	}
}

void PhysicsWorld::SetGravity(const glm::vec3& grav)
{
	m_gravity = grav;
	for (unsigned int i = 0; i, m_nonStaticRigidBodies.size(); ++i)
	{
		m_nonStaticRigidBodies[i]->SetGravity(grav);
	}
}

void PhysicsWorld::StepSimulation(float timeStep, int maxSubSteps, float fixedTimeStep)
{

	int numSimulationSubSteps = 0;

	if (maxSubSteps)
	{
		//fixed timestep with interpolation
		m_fixedTimeStep = fixedTimeStep;
		m_localTime += timeStep;
		if (m_localTime >= fixedTimeStep)
		{
			numSimulationSubSteps = int(m_localTime / fixedTimeStep);
			m_localTime -= numSimulationSubSteps * fixedTimeStep;
		}
	}
	else
	{
		//variable timestep
		fixedTimeStep = timeStep;
		m_localTime =  timeStep;
		m_fixedTimeStep = 0;
		if (std::fabsf(timeStep) < FLT_EPSILON)
		{
			numSimulationSubSteps = 0;
			maxSubSteps = 0;
		}
		else
		{
			numSimulationSubSteps = 1;
			maxSubSteps = 1;
		}
	}

	//process some debugging flags
	if (numSimulationSubSteps)
	{

		//clamp the number of substeps, to prevent simulation grinding spiralling down to a halt
		int clampedSimulationSteps = (numSimulationSubSteps > maxSubSteps) ? maxSubSteps : numSimulationSubSteps;

		//saveKinematicState(fixedTimeStep*clampedSimulationSteps);

		ApplyGravity();



		for (int i = 0; i<clampedSimulationSteps; i++)
		{
			SingleSimulationStep(fixedTimeStep);

			//internalSingleStepSimulation(fixedTimeStep);
			//synchronizeMotionStates();
		}

	}

	//TODO: ADD FLAG TO DISABLE DRAWING
	glm::vec3 aabbMin, aabbMax;
	glm::vec3 color(1.f, 0.f, 0.f);
	for (unsigned int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
	{
		m_nonStaticRigidBodies[i]->GetAABB(aabbMin, aabbMax);
		m_physDebugDrawer->DrawAABB(aabbMin, aabbMax, color);
	}

	ClearForces();
}

void PhysicsWorld::ApplyGravity()
{
	for (unsigned int i = 0; i< m_nonStaticRigidBodies.size(); ++i)
	{
		m_nonStaticRigidBodies[i]->ApplyGravity();
	}
}

void PhysicsWorld::PredictMotion(float timeStep)
{
	glm::vec3 linVel, angVel, totalF;
	for (unsigned int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
	{
		linVel = m_nonStaticRigidBodies[i]->GetLinearVelocity();
		angVel = m_nonStaticRigidBodies[i]->GetAngularVelocity();
		totalF = m_nonStaticRigidBodies[i]->GetTotalForce();

		//TODO MOVE TO RIGID BODY
		if (m_nonStaticRigidBodies[i]->GetMass() != 0)
			linVel += totalF / m_nonStaticRigidBodies[i]->GetMass() * timeStep;

		m_nonStaticRigidBodies[i]->GetInterpolationTransform() = 
			glm::translate(m_nonStaticRigidBodies[i]->GetInterpolationTransform(), linVel *timeStep);

	}
}

void PhysicsWorld::PerformMovement(float timeStep)
{
	glm::vec3 linVel, angVel, totalF;
	glm::mat4 trans, predictedTrans;
	for (unsigned int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
	{
		linVel = m_nonStaticRigidBodies[i]->GetLinearVelocity();
		angVel = m_nonStaticRigidBodies[i]->GetAngularVelocity();
		totalF = m_nonStaticRigidBodies[i]->GetTotalForce();

		trans = m_nonStaticRigidBodies[i]->GetTransform();
		if (m_nonStaticRigidBodies[i]->GetMass() != 0)
			linVel += totalF / m_nonStaticRigidBodies[i]->GetMass() * timeStep;
		
		predictedTrans = glm::translate(trans, linVel *timeStep);

		m_nonStaticRigidBodies[i]->UpdateTransform(predictedTrans);
		m_nonStaticRigidBodies[i]->SetLinearVelocity(linVel);

	}
}

bool CheckAABBIntersection(const glm::vec3& aabbMin1, const glm::vec3& aabbMax1, const glm::vec3& aabbMin2, const glm::vec3& aabbMax2)
{
	if (aabbMin1.x <= aabbMax2.x && aabbMax1.x >= aabbMin2.x &&
		aabbMin1.y <= aabbMax2.y && aabbMax1.y >= aabbMin2.y &&
		aabbMin1.z <= aabbMax2.z && aabbMax1.z >= aabbMin2.z)
		return true;
	return false;
}

void PhysicsWorld::PerformCollisionCheck()
{
	glm::vec3 aabbMin1, aabbMax1, aabbMin2, aabbMax2;

	for (unsigned int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
	{
		for (unsigned int j = 0; j < m_nonStaticRigidBodies.size(); ++j)
		{
			if (i == j)
				continue;

			m_nonStaticRigidBodies[i]->GetAABB(aabbMin1, aabbMax1);
			m_nonStaticRigidBodies[j]->GetAABB(aabbMin2, aabbMax2);

			if (CheckAABBIntersection(aabbMin1, aabbMax1, aabbMin2, aabbMax2))
			{
				float xDepth, yDepth, zDepth;

				m_nonStaticRigidBodies[i]->SetLinearVelocity(glm::vec3(0.f));
				m_nonStaticRigidBodies[i]->ApplyForce(glm::vec3(0, 9.8f, 0) * m_nonStaticRigidBodies[i]->GetMass());
			}
		}
	}
}

void PhysicsWorld::ClearForces()
{
	for (unsigned int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
	{
		m_nonStaticRigidBodies[i]->ClearForces();
	}
}

void PhysicsWorld::SingleSimulationStep(float fixedTimeStep)
{
	//Predict motion
	PredictMotion(fixedTimeStep);

	//Check collisions and apply
	PerformCollisionCheck();

	//Solve Constraints
	//SolveConstraints();

	//Perform movement
	PerformMovement(fixedTimeStep);
}

void PhysicsWorld::SetPhysDebugDrawer(PhysDebugDrawer* pdd)
{
	m_physDebugDrawer = pdd;
}