#include "NarrowphaseSAT.h"
#include "glm\gtc\matrix_transform.hpp"

void NarrowphaseSAT::PerformCollisionResolution(const std::vector<PhysicsDefs::CollisionPair>& collisionPairs)
{

	PhysicsDefs::AABB aabb1, aabb2;
	glm::vec3 normal, vel1, vel2, relativeVel, impulse, correction;
	glm::mat4 invTransform1;
	glm::mat4& interpolationTrans1 = glm::mat4(1);
	glm::mat4& interpolationTrans2 = glm::mat4(1);

	float depth, velAlongColNormal, minConst, totalSytemMass, mass1, mass2, invMass1, invMass2;
	for (int i = 0; i < collisionPairs.size(); ++i)
	{	
		interpolationTrans1 = collisionPairs[i].first->GetInterpolationTransform();
		interpolationTrans2 = collisionPairs[i].second->GetInterpolationTransform();

		aabb1 = collisionPairs[i].first->GetAABB();
		aabb2 = collisionPairs[i].second->GetAABB();
		aabb2.min = glm::vec3(glm::inverse(interpolationTrans1) * interpolationTrans2 * glm::vec4(aabb2.min, 1.f));
		aabb2.max = glm::vec3(glm::inverse(interpolationTrans1) * interpolationTrans2 * glm::vec4(aabb2.max, 1.f));
		if (SATDetectionAABB(aabb1, aabb2, normal, depth))
		{
			float xDepth, yDepth, zDepth;

			vel1 = collisionPairs[i].first->GetLinearVelocity();
			vel2 = collisionPairs[i].second->GetLinearVelocity();
			relativeVel = vel2 - vel1;
			velAlongColNormal = glm::dot(relativeVel, normal);

			if (velAlongColNormal < 0)
				continue;

			minConst = std::min(collisionPairs[i].first->GetRestitution(), collisionPairs[i].second->GetRestitution());
			mass1 = collisionPairs[i].first->GetMass();
			invMass1 = collisionPairs[i].first->GetInverseMass();
			mass2 = collisionPairs[i].second->GetMass();
			invMass2 = collisionPairs[i].second->GetInverseMass();
			totalSytemMass = mass1 + mass2;

			impulse = normal * (-(1 + minConst) * velAlongColNormal);

			collisionPairs[i].first->ApplyImpulse(-impulse * mass1 / totalSytemMass);
			collisionPairs[i].second->ApplyImpulse(impulse * mass2 / totalSytemMass);

			//Pushout to avoid sinking
			const float PERCENT = 1;
			const float THRESHOLD = 0.001;
			correction = std::max(depth - THRESHOLD, 0.0f) / (invMass1 + invMass2) * PERCENT * normal;
			interpolationTrans1 = glm::translate(interpolationTrans1, invMass1 * correction);
			interpolationTrans2 = glm::translate(interpolationTrans2, -invMass2 * correction);
			collisionPairs[i].first->UpdateInterpolationTransform(interpolationTrans1);
			collisionPairs[i].second->UpdateInterpolationTransform(interpolationTrans2);
		}
	}
	
}

bool NarrowphaseSAT::SATDetectionAABB(const PhysicsDefs::AABB& aabb1, const PhysicsDefs::AABB& aabb2, glm::vec3& normalOut, float& depthOut)
{
	depthOut = FLT_MAX;
	float currentAxisDepth = 0;
	bool test = true;

	if (aabb1.min.x <= aabb2.max.x && aabb1.max.x >= aabb2.min.x)
	{
		currentAxisDepth = std::min(aabb1.max.x, aabb2.max.x) - std::max(aabb1.min.x, aabb2.min.x);
		if (currentAxisDepth < depthOut)
		{
			depthOut = currentAxisDepth;
			normalOut = glm::vec3(1.f, 0, 0);
		}
	}
	else
		return false;

	if (aabb1.min.y <= aabb2.max.y && aabb1.max.y >= aabb2.min.y)
	{
		currentAxisDepth =std::min(aabb1.max.y, aabb2.max.y) - std::max(aabb1.min.y, aabb2.min.y);
		if (currentAxisDepth < depthOut)
		{
			depthOut = currentAxisDepth;
			normalOut = glm::vec3(0, 1.0f, 0);
		}
	}
	else
		return false;

	if (aabb1.min.z <= aabb2.max.x && aabb1.max.z >= aabb2.min.z)
	{
		currentAxisDepth =std::min(aabb1.max.z, aabb2.max.z) - std::max(aabb1.min.z, aabb2.min.z);
		if (currentAxisDepth < depthOut)
		{
			depthOut = currentAxisDepth;
			normalOut = glm::vec3(0, 0, 1.f);
		}
	}
	else
		return false;

	if (depthOut < FLT_EPSILON)
		return false;

	return true;
}