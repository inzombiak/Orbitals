#include "PhysicsWorld.h"
#include "glm\gtc\matrix_transform.hpp"
#include "../Rendering/PhysDebugDrawer.h"
#include "IBroadphase.h"
#include "INarrowphase.h"

PhysicsWorld::PhysicsWorld(IBroadphase* broadphase, INarrowphase* narrowphase)
{
	if (!broadphase || !narrowphase)
		assert(0);
	m_gravity = glm::vec3(0);
	m_broadphase = broadphase;
	m_narrowphase = narrowphase;
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
	glm::vec4 aabbMin4, aabbMax4;
	glm::vec3 color(1.f, 0.f, 0.f);
	for (unsigned int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
	{
		aabb = m_nonStaticRigidBodies[i]->GetAABB();
		auto transform = m_nonStaticRigidBodies[i]->GetTransform();
		aabbMin4 = transform * glm::vec4(aabb.min, 1.f);
		aabbMax4 = transform * glm::vec4(aabb.max, 1.f);
		aabb.min = glm::vec3(aabbMin4);
		aabb.max = glm::vec3(aabbMax4);
		m_physDebugDrawer->DrawAABB(aabb.min, aabb.max, color);
	}

	if (m_narrowphaseError.size() > 1)
	{
		for (int i = 0; i < m_narrowphaseError.size(); ++i)
		{
			m_physDebugDrawer->DrawLine(m_narrowphaseError[i], m_narrowphaseError[(i + 1) % m_narrowphaseError.size()], glm::vec3(1.f, 1.f, 0.f));
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

		trans = m_nonStaticRigidBodies[i]->GetInterpolationTransform();
		
		predictedTrans = glm::translate(trans, linVel *timeStep);
		m_nonStaticRigidBodies[i]->UpdateTransform(predictedTrans);
	}
}

void PhysicsWorld::PerformCollisionCheck()
{
	//Do broadphase
	auto collidingPairs = m_broadphase->GetCollisionPairs();

	if (collidingPairs.size() == 0)
		return;

	auto narrowPhaseResult = m_narrowphase->PerformCollisionResolution(collidingPairs, NarrowphaseErrorCalback);

	//TODO: THIS RESULT MIGHT NOT HAVE THE RIGHT LOCAL AND WORLD POINTS IN A AND B, TEST IT

	if (narrowPhaseResult.size() < 1)
		return;

	float xDepth, yDepth, zDepth;
	glm::vec3 vel1, vel2, relativeVel, impulse, correction;
	glm::mat4 invTransform1;
	glm::mat4 interpolationTrans1;
	glm::mat4 interpolationTrans2;

	float velAlongColNormal, minConst, totalSytemMass, mass1, mass2, invMass1, invMass2;
	for (int i = 0; i < narrowPhaseResult.size(); ++i)
	{
		interpolationTrans1 = narrowPhaseResult[i].first.first->GetInterpolationTransform();
		interpolationTrans2 = narrowPhaseResult[i].first.second->GetInterpolationTransform();
		vel1 = narrowPhaseResult[i].first.first->GetLinearVelocity();
		vel2 = narrowPhaseResult[i].first.second->GetLinearVelocity();

		relativeVel = vel2 - vel1;
		narrowPhaseResult[i].second.tangent1 = glm::normalize(relativeVel);
		narrowPhaseResult[i].second.tangent2 = glm::cross(narrowPhaseResult[i].second.tangent1, narrowPhaseResult[i].second.normal);
		velAlongColNormal = glm::dot(relativeVel, narrowPhaseResult[i].second.normal);

		if (velAlongColNormal < 0)
			continue;

		minConst = std::min(narrowPhaseResult[i].first.first->GetRestitution(), narrowPhaseResult[i].first.second->GetRestitution());
		mass1 = narrowPhaseResult[i].first.first->GetMass();
		invMass1 = narrowPhaseResult[i].first.first->GetInverseMass();
		mass2 = narrowPhaseResult[i].first.second->GetMass();
		invMass2 = narrowPhaseResult[i].first.second->GetInverseMass();
		totalSytemMass = mass1 + mass2;

		impulse = narrowPhaseResult[i].second.normal * (-(1 + minConst) * velAlongColNormal);

		narrowPhaseResult[i].first.first->ApplyImpulse(-impulse * mass1 / totalSytemMass);
		narrowPhaseResult[i].first.second->ApplyImpulse(impulse * mass2 / totalSytemMass);

		//Pushout to avoid sinking
		const float PERCENT = 1;
		const float THRESHOLD = 0.001;
		correction = std::max(narrowPhaseResult[i].second.depth - THRESHOLD, 0.0f) / (invMass1 + invMass2) * PERCENT * narrowPhaseResult[i].second.normal;
		interpolationTrans1 = glm::translate(interpolationTrans1, invMass1 * correction);
		interpolationTrans2 = glm::translate(interpolationTrans2, -invMass2 * correction);
		narrowPhaseResult[i].first.first->UpdateInterpolationTransform(interpolationTrans1);
		narrowPhaseResult[i].first.second->UpdateInterpolationTransform(interpolationTrans2);

		//Apply friction
		float frict1 = narrowPhaseResult[i].first.first->GetFriction();
		float frict2 = narrowPhaseResult[i].first.second->GetFriction();

		//collisionPairs[i].first->SetLinearVelocity(glm::vec3(0, 0, 0));
		//collisionPairs[i].second->SetLinearVelocity(glm::vec3(0, 0, 0));
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

std::vector<glm::vec3> PhysicsWorld::m_narrowphaseError;