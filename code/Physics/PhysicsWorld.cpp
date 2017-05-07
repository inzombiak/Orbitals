#include "PhysicsWorld.h"
#include "glm\gtc\matrix_transform.hpp"
#include "../Rendering/PhysDebugDrawer.h"
#include "IBroadphase.h"
#include "INarrowphase.h"
#include "IConstraintSolver.h"

PhysicsWorld::PhysicsWorld(IBroadphase* broadphase, INarrowphase* narrowphase, IConstraintSolver* constraintSolver)
{
	if (!broadphase || !narrowphase)
		assert(0);
	m_gravity = glm::vec3(0);
	m_broadphase = broadphase;
	m_narrowphase = narrowphase;
	m_constraintSolver = constraintSolver;
}

void PhysicsWorld::AddRigidBody(IRigidBody* body)
{
	body->SetGravity(m_gravity);
	m_nonStaticRigidBodies.push_back(body);
	m_broadphase->AddAABB(&body->GetAABB());
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
	PhysicsDefs::AABB aabb;
	PhysicsDefs::OBB obb;
	glm::vec3 color(1.f, 0.f, 0.f), colorOBB(0.f, 0.f, 1.f);
	for (unsigned int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
	{
		aabb = m_nonStaticRigidBodies[i]->GetAABB();
		auto transform = m_nonStaticRigidBodies[i]->GetTransform();
		aabb.min = glm::vec3(transform[3]) + aabb.min;
		aabb.max = glm::vec3(transform[3]) + aabb.max;
		m_physDebugDrawer->DrawAABB(aabb.min, aabb.max, color);

		obb = m_nonStaticRigidBodies[i]->GetOBB();

		glm::vec3 currentVertex(-1, -1, -1);
		glm::vec3 x, y, z;
		glm::vec3 from, to;
		glm::mat3 axes(obb.localX, obb.localY, glm::cross(obb.localX, obb.localY));
		static const glm::vec3 OFFSET(-0.01f, -0.01f, -0.01f);
		for (unsigned int i = 0; i < 4; ++i)
		{
			x = obb.localX * obb.halfExtents.x * currentVertex.x;
			y = obb.localY * obb.halfExtents.y * currentVertex.y;
			z = axes[2] * obb.halfExtents.z * currentVertex.z;

			from = x + y + z;
			from += obb.pos;

			for (unsigned int j = 0; j < 3; ++j)
			{
				to = from;
				to -= 2.f * (currentVertex[j] * obb.halfExtents[j] * axes[j]);

				m_physDebugDrawer->DrawLine(from, to, colorOBB);
			}

			currentVertex[(i % 2) + 1] *= -1;
			currentVertex[((i + 1) % 2) * 2] *= -1;
		}
	}

	if (m_narrowphaseError.size() > 1)
	{
		for (int i = 0; i < m_narrowphaseError.size(); ++i)
		{
			m_physDebugDrawer->DrawLine(m_narrowphaseError[i], m_narrowphaseError[(i + 1) % m_narrowphaseError.size()], glm::vec3(1.f, 1.f, 0.f));
		}
	}

	PhysicsDefs::ContactInfo info;
	PhysicsDefs::CollisionPair bodies;
	if (m_narrowphaseResult.size() > 0)
	{
		for (unsigned int i = 0; i < m_narrowphaseResult.size(); ++i)
		{
			info = m_narrowphaseResult[i].second;
			bodies = m_narrowphaseResult[i].first;

			//m_physDebugDrawer->DrawPoint(info.worldPos, 2, glm::vec3(0, 1, 1));
			m_physDebugDrawer->DrawPoint(info.worldPointA, 1, glm::vec3(0, 1, 1));
			//m_physDebugDrawer->DrawPoint(info.worldPointB, 2, glm::vec3(1, 0, 1));
			//m_physDebugDrawer->DrawPoint(glm::vec3(bodies.first->GetTransform() * glm::vec4(info.localPointA, 1)), 3, glm::vec3(1, 1, 0));
			m_physDebugDrawer->DrawPoint(glm::vec3(bodies.second->GetTransform() * glm::vec4(info.localPointB, 1)), 4, glm::vec3(1, 0, 0));

		}
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

		m_nonStaticRigidBodies[i]->UpdateInterpolationTransform( 
			glm::translate(m_nonStaticRigidBodies[i]->GetTransform(), linVel *timeStep));
		m_nonStaticRigidBodies[i]->SetLinearVelocity(linVel);

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
		
		predictedTrans = glm::translate(trans, linVel *timeStep);
		m_nonStaticRigidBodies[i]->UpdateTransform(predictedTrans);
	}
}

void PhysicsWorld::PerformCollisionCheck(float dt)
{
	//Do broadphase
	auto collidingPairs = m_broadphase->GetCollisionPairs();

	if (collidingPairs.size() == 0)
		return;

	m_narrowphaseResult = m_narrowphase->CheckCollision(collidingPairs, NarrowphaseErrorCalback);

	//TODO: THIS RESULT MIGHT NOT HAVE THE RIGHT LOCAL AND WORLD POINTS IN A AND B, TEST IT

	if (m_narrowphaseResult.size() < 1)
		return;

	m_constraintSolver->SolveConstraints(m_narrowphaseResult, dt);
}

void PhysicsWorld::NarrowphaseErrorCalback(std::vector<glm::vec3> finalResult)
{
	m_narrowphaseError = finalResult;
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
	PerformCollisionCheck(fixedTimeStep);

	//Solve Constraints
	//SolveConstraints();

	//Perform movement
	PerformMovement(fixedTimeStep);
}

void PhysicsWorld::SetPhysDebugDrawer(PhysDebugDrawer* pdd)
{
	m_physDebugDrawer = pdd;
}

std::vector<glm::vec3> PhysicsWorld::m_narrowphaseError;