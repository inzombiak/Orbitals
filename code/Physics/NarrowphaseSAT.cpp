#include "NarrowphaseSAT.h"
#include "glm\gtc\matrix_transform.hpp"

std::vector<PhysicsDefs::CollPairContactInfo> NarrowphaseSAT::PerformCollisionResolution(const std::vector<PhysicsDefs::CollisionPair>& collisionPairs, ErrorCallBack ecb)
{

	PhysicsDefs::AABB aabb1, aabb2;
	glm::vec3 normal, vel1, vel2, relativeVel, impulse, correction;

	glm::mat4& interpolationTrans1 = glm::mat4(1);
	glm::mat4& interpolationTrans2 = glm::mat4(1);
	glm::mat4 bodyBtoA; 
	std::vector<PhysicsDefs::CollPairContactInfo> result;
	PhysicsDefs::ContactInfo contactInfo;

	float depth, velAlongColNormal, minConst, totalSytemMass, mass1, mass2, invMass1, invMass2;
	for (int i = 0; i < collisionPairs.size(); ++i)
	{	
		interpolationTrans1 = collisionPairs[i].first->GetInterpolationTransform();
		interpolationTrans2 = collisionPairs[i].second->GetInterpolationTransform();
		bodyBtoA = glm::inverse(interpolationTrans1) * interpolationTrans2;
		aabb1 = collisionPairs[i].first->GetAABB();
		aabb2 = collisionPairs[i].second->GetAABB();
		aabb2.min = glm::vec3(bodyBtoA * glm::vec4(aabb2.min, 1.f));
		aabb2.max = glm::vec3(bodyBtoA * glm::vec4(aabb2.max, 1.f));
		if (SATDetectionAABB(aabb1, aabb2, contactInfo))
		{
			contactInfo.worldPointA = glm::vec3(interpolationTrans1 *  glm::vec4(contactInfo.localPointA, 1));
			//Since we moved everything into A's space we neeed to move it out 
			contactInfo.worldPointB = glm::vec3(interpolationTrans1 *  glm::vec4(contactInfo.localPointA + contactInfo.normal * contactInfo.depth, 1.f));
			contactInfo.localPointB = glm::vec3(glm::inverse(interpolationTrans2) *  glm::vec4(contactInfo.worldPointB, 1.f));
			
			result.push_back(std::make_pair(collisionPairs[i], contactInfo));
		}
	}

	return result;
	
}

bool NarrowphaseSAT::SATDetectionAABB(const PhysicsDefs::AABB& aabb1, const PhysicsDefs::AABB& aabb2, PhysicsDefs::ContactInfo& contactInfo)
{
	contactInfo.depth = FLT_MAX;
	float currentAxisDepth = 0;
	bool test = true;
	int sign;

	if (aabb1.min.x <= aabb2.max.x && aabb1.max.x >= aabb2.min.x)
	{
		currentAxisDepth = std::min(aabb1.max.x, aabb2.max.x) - std::max(aabb1.min.x, aabb2.min.x);
		if (currentAxisDepth < contactInfo.depth)
		{
			contactInfo.depth = currentAxisDepth;
			contactInfo.normal = glm::vec3(1.f, 0, 0);
			sign = (aabb2.min.x - aabb1.min.x < 0) ? -1 : 1;
			contactInfo.normal *= sign;

			contactInfo.localPointA = aabb1.max.x - contactInfo.normal * contactInfo.depth;
		}
	}
	else
		return false;

	if (aabb1.min.y <= aabb2.max.y && aabb1.max.y >= aabb2.min.y)
	{
		currentAxisDepth =std::min(aabb1.max.y, aabb2.max.y) - std::max(aabb1.min.y, aabb2.min.y);
		if (currentAxisDepth < contactInfo.depth)
		{
			contactInfo.depth = currentAxisDepth;
			contactInfo.normal = glm::vec3(0, 1.0f, 0);
			sign = (aabb2.min.y - aabb1.min.y < 0) ? -1 : 1;
			contactInfo.normal *= sign;

			contactInfo.localPointA = aabb1.max.y - contactInfo.normal * contactInfo.depth;
		}
	}
	else
		return false;

	if (aabb1.min.z <= aabb2.max.x && aabb1.max.z >= aabb2.min.z)
	{
		currentAxisDepth = std::min(aabb1.max.z, aabb2.max.z) - std::max(aabb1.min.z, aabb2.min.z);
		if (currentAxisDepth < contactInfo.depth)
		{
			contactInfo.depth = currentAxisDepth;
			contactInfo.normal = glm::vec3(0, 0, 1.f);
			sign = (aabb2.min.z - aabb1.min.z < 0) ? -1 : 1;
			contactInfo.normal *= sign;

			contactInfo.localPointA = aabb1.max.z - contactInfo.normal * contactInfo.depth;
		}
	}
	else
		return false;

	if (contactInfo.depth < FLT_EPSILON)
		return false;

	//contactInfo.localPointA = 

	return true;
}