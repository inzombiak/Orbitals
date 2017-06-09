#include "PhysicsWorld.h"
#include "../Rendering/PhysDebugDrawer.h"
#include "IBroadphase.h"
#include "INarrowphase.h"
#include "IConstraintSolver.h"
#include "../Utilities/Debug.h"
#include "glm\gtc\matrix_transform.hpp"
#include "glm\gtx\quaternion.hpp"

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
		aabb.min = transform.GetOrigin() + aabb.min;
		aabb.max = transform.GetOrigin() + aabb.max;
		m_physDebugDrawer->DrawAABB(aabb.min, aabb.max, color);

		obb = m_nonStaticRigidBodies[i]->GetOBB();

		glm::vec3 currentVertex(-1, -1, -1);
		glm::vec3 x, y, z;
		glm::vec3 from, to;
	
		static const glm::vec3 OFFSET(-0.01f, -0.01f, -0.01f);
		for (unsigned int i = 0; i < 4; ++i)
		{
			x = obb.localAxes[0] * obb.halfExtents.x * currentVertex.x;
			y = obb.localAxes[1] * obb.halfExtents.y * currentVertex.y;
			z = obb.localAxes[2] * obb.halfExtents.z * currentVertex.z;

			from = x + y + z;
			from += obb.pos;

			for (unsigned int j = 0; j < 3; ++j)
			{
				to = from;
				to -= 2.f * (currentVertex[j] * obb.halfExtents[j] * obb.localAxes[j]);

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

	glm::vec3 pos1, pos2;
	PhysicsDefs::ContactInfo ci;
	if (m_manifolds.size() > 0)
	{
		for (unsigned int i = 0; i < m_manifolds.size(); ++i)
		{
			pos1 = m_manifolds[i].m_bodyA->GetInterpolationTransform().GetOrigin();
			pos2 = m_manifolds[i].m_bodyB->GetInterpolationTransform().GetOrigin();
			m_physDebugDrawer->DrawPoint(pos1, 0.2, glm::vec3(0, 1, 0));
			m_physDebugDrawer->DrawPoint(pos2, 0.5, glm::vec3(0, 0, 1));
			for (int j = 0; j < m_manifolds[i].m_contactCount; ++j)
			{
				ci = m_manifolds[i].m_contacts[j];
				m_physDebugDrawer->DrawPoint(ci.worldPos, 0.2, glm::vec3(0, 1, 1));
				m_physDebugDrawer->DrawLine(pos1, pos1 + ci.localPointA, glm::vec3(0.5, 0.5, 0.5));
				m_physDebugDrawer->DrawLine(pos2, pos2 + ci.localPointB, glm::vec3(0.5, 0.f, 0.5));
			}
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
	float angMag;
	glm::vec3 linVel, angVel, totalF, pos;
	OTransform trans, finalTrans;
	glm::quat rot;
	for (unsigned int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
	{
		linVel = m_nonStaticRigidBodies[i]->GetLinearVelocity();
		angVel = m_nonStaticRigidBodies[i]->GetAngularVelocity();
		totalF = m_nonStaticRigidBodies[i]->GetTotalForce();
		angMag = glm::length(angVel);

		if (m_nonStaticRigidBodies[i]->GetMass() != 0)
			linVel += totalF / m_nonStaticRigidBodies[i]->GetMass() * timeStep;

		trans = m_nonStaticRigidBodies[i]->GetTransform();
		rot = trans.GetRotation();
		pos = trans.GetOrigin();

		rot *= glm::quat(angVel* timeStep);
		pos += linVel * timeStep;
		rot = glm::normalize(rot);
		trans.SetOrigin(pos);
		trans.SetRotation(rot);

		m_nonStaticRigidBodies[i]->UpdateInterpolationTransform(trans);
		m_nonStaticRigidBodies[i]->SetLinearVelocity(linVel);

	}
}

void PhysicsWorld::PerformMovement(float timeStep)
{
	float angMag;
	glm::vec3 linVel, angVel, pos;
	OTransform trans, finalTrans;
	glm::quat rot;
	for (unsigned int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
	{
		linVel = m_nonStaticRigidBodies[i]->GetLinearVelocity() ;
		angVel = m_nonStaticRigidBodies[i]->GetAngularVelocity();
		angMag = glm::length(angVel);

		if (glm::dot(linVel, linVel) < 0.01)
		{
			linVel = glm::vec3(0);
			m_nonStaticRigidBodies[i]->SetLinearVelocity(linVel);
		}
		if (glm::dot(angVel, angVel) < 0.01)
		{
			angVel = glm::vec3(0);
			m_nonStaticRigidBodies[i]->SetAngularVelocity(angVel);
		}

		trans = m_nonStaticRigidBodies[i]->GetTransform();

		rot = trans.GetRotation();
		pos = trans.GetOrigin();

		rot *= glm::quat(angVel* timeStep);
		pos += linVel * timeStep;
		rot = glm::normalize(rot);
		trans.SetOrigin(pos);
		trans.SetRotation(rot);

		m_nonStaticRigidBodies[i]->UpdateTransform(trans);
	}
}

void PhysicsWorld::PerformCollisionCheck(float dt)
{
	auto collidingPairs = m_broadphase->GetCollisionPairs();

	if (collidingPairs.size() != 0)
	{
		m_manifolds = m_narrowphase->CheckCollision(collidingPairs, NarrowphaseErrorCalback);
		if (m_manifolds.size() > 0)
			m_constraintSolver->SolveConstraints2(m_manifolds, dt);
	}
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