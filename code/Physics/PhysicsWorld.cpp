#include "PhysicsWorld.h"
#include "glm\gtc\matrix_transform.hpp"
#include "../Rendering/PhysDebugDrawer.h"
#include "GJKSolver.h"
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
	glm::vec4 aabbMin4, aabbMax4;
	glm::vec3 color(1.f, 0.f, 0.f);
	for (unsigned int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
	{
		m_nonStaticRigidBodies[i]->GetAABB(aabbMin, aabbMax);
		auto transform = m_nonStaticRigidBodies[i]->GetTransform();
		aabbMin4 = transform * glm::vec4(aabbMin, 1.f);
		aabbMax4 = transform * glm::vec4(aabbMax, 1.f);
		aabbMin = glm::vec3(aabbMin4);
		aabbMax = glm::vec3(aabbMax4);
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
			glm::translate(m_nonStaticRigidBodies[i]->GetTransform(), linVel *timeStep);
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

bool CheckAABBIntersection(const glm::vec3& aabbMin1, const glm::vec3& aabbMax1, const glm::vec3& aabbMin2, const glm::vec3& aabbMax2)
{
	if (aabbMin1.x <= aabbMax2.x && aabbMax1.x >= aabbMin2.x &&
		aabbMin1.y <= aabbMax2.y && aabbMax1.y >= aabbMin2.y &&
		aabbMin1.z <= aabbMax2.z && aabbMax1.z >= aabbMin2.z)
	{
		return true;
	}

	return false;
}

bool SATDetectionAABB(const glm::vec3& aabbMin1, const glm::vec3& aabbMax1, const glm::vec3& aabbMin2, const glm::vec3& aabbMax2,
	glm::vec3& normalOut, float& depthOut)
{
	depthOut = FLT_MAX;
	float currentAxisDepth = 0;
	bool test = true;

	if (aabbMin1.x <= aabbMax2.x && aabbMax1.x >= aabbMin2.x)
	{
		currentAxisDepth = std::max(0.f, std::min(aabbMax2.x, aabbMax2.x) - std::max(aabbMin1.x, aabbMin2.x));
		if (currentAxisDepth < depthOut)
		{
			depthOut = currentAxisDepth;
			normalOut = glm::vec3(1.f, 0 , 0);
		}
	}
	else
		return false;

	if (aabbMin1.y <= aabbMax2.y && aabbMax1.y >= aabbMin2.y)
	{
		currentAxisDepth = std::max(0.f, std::min(aabbMax2.y, aabbMax2.y) - std::max(aabbMin1.y, aabbMin2.y));
		if (currentAxisDepth < depthOut)
		{
			depthOut = currentAxisDepth;
			normalOut = glm::vec3(0, 1.0f, 0);
		}
	}
	else
		return false;

	if (aabbMin1.z <= aabbMax2.z && aabbMax1.z >= aabbMin2.z)
	{
		currentAxisDepth = std::max(0.f, std::min(aabbMax2.z, aabbMax2.z) - std::max(aabbMin1.z, aabbMin2.z));
		if (currentAxisDepth < depthOut)
		{
			depthOut = currentAxisDepth;
			normalOut = glm::vec3(0, 0, 1.f);
		}
	}
	else
		return false;

	return true;
}

bool DoGJKAABB(const glm::vec3& aabbMin1, const glm::vec3& aabbMax1, const glm::vec3& aabbMin2, const glm::vec3& aabbMax2)
{
	std::vector<glm::vec3> aabb1, aabb2;

	aabb1.push_back(glm::vec3(aabbMin1.x, aabbMin1.y, aabbMin1.z));
	aabb1.push_back(glm::vec3(aabbMin1.x, aabbMax1.y, aabbMin1.z));
	aabb1.push_back(glm::vec3(aabbMin1.x, aabbMin1.y, aabbMax1.z));
	aabb1.push_back(glm::vec3(aabbMin1.x, aabbMax1.y, aabbMax1.z));
	aabb1.push_back(glm::vec3(aabbMax1.x, aabbMin1.y, aabbMin1.z));
	aabb1.push_back(glm::vec3(aabbMax1.x, aabbMax1.y, aabbMin1.z));
	aabb1.push_back(glm::vec3(aabbMax1.x, aabbMin1.y, aabbMax1.z));
	aabb1.push_back(glm::vec3(aabbMax1.x, aabbMax1.y, aabbMax1.z));

	aabb2.push_back(glm::vec3(aabbMax2.x, aabbMin2.y, aabbMin2.z));
	aabb2.push_back(glm::vec3(aabbMax2.x, aabbMax2.y, aabbMin2.z));
	aabb2.push_back(glm::vec3(aabbMax2.x, aabbMin2.y, aabbMax2.z));
	aabb2.push_back(glm::vec3(aabbMax2.x, aabbMax2.y, aabbMax2.z));
	aabb2.push_back(glm::vec3(aabbMin2.x, aabbMin2.y, aabbMin2.z));
	aabb2.push_back(glm::vec3(aabbMin2.x, aabbMax2.y, aabbMin2.z));
	aabb2.push_back(glm::vec3(aabbMin2.x, aabbMin2.y, aabbMax2.z));
	aabb2.push_back(glm::vec3(aabbMin2.x, aabbMax2.y, aabbMax2.z));


	PhysicsDefs::Contact data;
	
	return GJKSolver::CheckCollision(aabb1, aabb2, data);
}

void PhysicsWorld::PerformCollisionCheck()
{
	glm::vec3 aabbMin1, aabbMax1, aabbMin2, aabbMax2, normal, vel1, vel2, relativeVel, impulse, correction;
	glm::vec4 aabbMin2_4, aabbMax2_4;
	glm::mat4 invTransform1;
	glm::mat4& interpolationTrans1 = glm::mat4(1);
	glm::mat4& interpolationTrans2 = glm::mat4(1);

	float depth, velAlongColNormal, minConst, totalSytemMass, mass1, mass2, invMass1, invMass2;

	for (unsigned int i = 0; i < m_nonStaticRigidBodies.size(); ++i)
	{
		m_nonStaticRigidBodies[i]->GetAABB(aabbMin1, aabbMax1);
		interpolationTrans1 = m_nonStaticRigidBodies[i]->GetInterpolationTransform();
		invTransform1 = glm::inverse(interpolationTrans1);
		vel1 = m_nonStaticRigidBodies[i]->GetLinearVelocity();
		mass1 = m_nonStaticRigidBodies[i]->GetMass();
		invMass1 = m_nonStaticRigidBodies[i]->GetInverseMass();

		for (unsigned int j = i + 1; j < m_nonStaticRigidBodies.size(); ++j)
		{
			m_nonStaticRigidBodies[j]->GetAABB(aabbMin2, aabbMax2);
			interpolationTrans2 = m_nonStaticRigidBodies[j]->GetInterpolationTransform();
			aabbMin2_4 = invTransform1 * interpolationTrans2 * glm::vec4(aabbMin2, 1.f);
			aabbMax2_4 = invTransform1 * interpolationTrans2 * glm::vec4(aabbMax2, 1.f);
			aabbMin2 = glm::vec3(aabbMin2_4);
			aabbMax2 = glm::vec3(aabbMax2_4);
			if (DoGJKAABB(aabbMin1, aabbMax1, aabbMin2, aabbMax2))
			{
				m_nonStaticRigidBodies[i]->SetLinearVelocity(glm::vec3(0,0,0));
			}

	//		
	//		
	//		
	//		
	//		

	//		if (SATDetectionAABB(aabbMin1, aabbMax1, aabbMin2, aabbMax2, normal, depth))
	//		{
	//			float xDepth, yDepth, zDepth;

	//			vel2 = m_nonStaticRigidBodies[j]->GetLinearVelocity();
	//			relativeVel = vel2 - vel1;
	//			velAlongColNormal = glm::dot(relativeVel, normal);

	//			if (velAlongColNormal < 0)
	//				continue;

	//			minConst = std::min(m_nonStaticRigidBodies[i]->GetRestitution(), m_nonStaticRigidBodies[j]->GetRestitution());
	//			mass2 = m_nonStaticRigidBodies[j]->GetMass();
	//			invMass2 = m_nonStaticRigidBodies[j]->GetInverseMass();
	//			totalSytemMass = mass1 + mass2;

	//			impulse = normal * (-(1 + minConst) * velAlongColNormal* 10);

	//			m_nonStaticRigidBodies[i]->ApplyImpulse(-impulse * mass1 / totalSytemMass);
	//			m_nonStaticRigidBodies[j]->ApplyImpulse(impulse * mass2 / totalSytemMass);

	//			//Pushout to avoid sinking
	//			const float PERCENT = 1;
	//			const float THRESHOLD = 0.001;
	//			correction = std::max(depth - THRESHOLD, 0.0f) / (invMass1 + invMass2) * PERCENT * normal;
	//			interpolationTrans1 = glm::translate(interpolationTrans1, invMass1 * correction);
	//			interpolationTrans2 = glm::translate(interpolationTrans2, -invMass2 * correction);
	//			m_nonStaticRigidBodies[i]->GetInterpolationTransform() = interpolationTrans1;
	//			m_nonStaticRigidBodies[j]->GetInterpolationTransform() = interpolationTrans2;
	//		}
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