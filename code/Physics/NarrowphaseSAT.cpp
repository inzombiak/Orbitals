#include "NarrowphaseSAT.h"
#include "glm\gtc\matrix_transform.hpp"

std::vector<PhysicsDefs::CollPairContactInfo> NarrowphaseSAT::CheckCollision(const std::vector<PhysicsDefs::CollisionPair>& collisionPairs, ErrorCallBack ecb)
{

	std::vector<PhysicsDefs::CollPairContactInfo> result;
	PhysicsDefs::ContactInfo contactInfo;

	float depth, velAlongColNormal, minConst, totalSytemMass, mass1, mass2, invMass1, invMass2;
	for (int i = 0; i < collisionPairs.size(); ++i)
	{	
		
		//if (SATDetectionAABB(collisionPairs[i].first, collisionPairs[i].second, contactInfo))
		//{
		//	//Since we moved everything into A's space we neeed to move it out 
		//	result.push_back(std::make_pair(collisionPairs[i], contactInfo));
		//}

		if (SATDetectionOBB(collisionPairs[i].first, collisionPairs[i].second, contactInfo))
		{
			//Since we moved everything into A's space we neeed to move it out 
			result.push_back(std::make_pair(collisionPairs[i], contactInfo));
		}
	}

	return result;
	
}
//
//bool NarrowphaseSAT::SATDetectionAABB(const PhysicsDefs::AABB& aabb1, const PhysicsDefs::AABB& aabb2, PhysicsDefs::ContactInfo& contactInfo)
//{
//
//}

bool NarrowphaseSAT::SATDetectionAABB(IRigidBody* body1, IRigidBody* body2, PhysicsDefs::ContactInfo& contactInfo)
{
	contactInfo.depth = FLT_MAX;
	float currentAxisDepth = 0;
	bool test = true;
	int sign;
	glm::vec3 contactPointA, contactNormal;

	PhysicsDefs::AABB aabb1, aabb2;

	aabb1 = body1->GetAABB();
	aabb2 = body2->GetAABB();

	glm::mat4 interpolationTrans1 = aabb1.worldTransform;
	glm::mat4 interpolationTrans2 = aabb2.worldTransform;
	glm::mat4 bodyBtoA = glm::inverse(interpolationTrans1) * interpolationTrans2;

	aabb2.min = glm::vec3(bodyBtoA * glm::vec4(aabb2.min, 1.f));
	aabb2.max = glm::vec3(bodyBtoA * glm::vec4(aabb2.max, 1.f));

	if (aabb1.min.x <= aabb2.max.x && aabb1.max.x >= aabb2.min.x)
	{
		currentAxisDepth = std::min(aabb1.max.x, aabb2.max.x) - std::max(aabb1.min.x, aabb2.min.x);
		contactNormal = glm::vec3(1.f, 0, 0);
		sign = (aabb2.min.x - aabb1.min.x < 0) ? -1 : 1;
		
		contactPointA.x = sign * (currentAxisDepth);
		contactNormal *= sign;

		if (currentAxisDepth < contactInfo.depth)
		{
			contactInfo.depth = currentAxisDepth;
			contactInfo.normal = contactNormal;
		}
	}
	else
		return false;

	if (aabb1.min.y <= aabb2.max.y && aabb1.max.y >= aabb2.min.y)
	{
		currentAxisDepth = std::min(aabb1.max.y, aabb2.max.y) - std::max(aabb1.min.y, aabb2.min.y);
		contactNormal = glm::vec3(0, 1.0f, 0);
		sign = (aabb2.min.y - aabb1.min.y < 0) ? -1 : 1;
		
		contactPointA.y = sign * (aabb1.max.y - currentAxisDepth);
		contactNormal *= sign;

		if (currentAxisDepth < contactInfo.depth)
		{
			contactInfo.depth = currentAxisDepth;
			contactInfo.normal = contactNormal;
		}
	}
	else
		return false;

	if (aabb1.min.z <= aabb2.max.x && aabb1.max.z >= aabb2.min.z)
	{
		currentAxisDepth = std::min(aabb1.max.z, aabb2.max.z) - std::max(aabb1.min.z, aabb2.min.z);
		contactNormal = glm::vec3(0, 0, 1.f);
		sign = (aabb2.min.z - aabb1.min.z < 0) ? -1 : 1;

		contactPointA.z = sign * (currentAxisDepth);
		contactNormal *= sign;

		if (currentAxisDepth < contactInfo.depth)
		{
			contactInfo.depth = currentAxisDepth;
			contactInfo.normal = contactNormal;
		}
	}
	else
		return false;

	if (contactInfo.depth < FLT_EPSILON)
		return false;

	contactInfo.localPointA = contactPointA;

	//contactInfo.worldPointB = glm::vec3(interpolationTrans1 *  glm::vec4(contactInfo.localPointA - contactInfo.normal * contactInfo.depth, 1.f));

	//contactInfo.localPointA = glm::vec3(glm::inverse(interpolationTrans2) *  glm::vec4(contactInfo.worldPointB, 1.f));
	contactInfo.worldPointA = glm::vec3(interpolationTrans1 * glm::vec4(contactInfo.localPointA, 1));

	return true;
}

//Based on Real-time Collision Detection by Christer Ericson, pg. 103
bool NarrowphaseSAT::SATDetectionOBB(IRigidBody* body1, IRigidBody* body2, PhysicsDefs::ContactInfo& contactInfo)
{

	float ra, rb, depth;
	contactInfo.depth = FLT_MAX;
	glm::mat3 R, absR;

	PhysicsDefs::OBB obb1 = body1->GetOBB();
	PhysicsDefs::OBB obb2 = body2->GetOBB();

	//TODO: WHY?
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			R[i][j] = glm::dot(obb1.localAxes[j], obb2.localAxes[j]);
			absR[i][j] = std::abs(R[i][j]) + FLT_EPSILON;
		}
	}

	glm::vec3 trans = obb2.pos - obb1.pos;
	trans = glm::vec3(glm::dot(trans, obb1.localAxes[0]), glm::dot(trans, obb1.localAxes[1]), glm::dot(trans, obb1.localAxes[2]));

	//for (int i = 0; i < 3; ++i)
	//{
	//	for (int j = 0; j < 3; ++j)
	//	{
	//		absR[i][j] = std::abs(R[i][j]) + FLT_EPSILON;
	//	}
	//}

	//Text A0, A1 and A2
	for (int i = 0; i < 3; ++i)
	{
		ra = obb1.halfExtents[i];
		rb = obb2.halfExtents[0] * absR[i][0] + obb2.halfExtents[1] * absR[i][1] + obb2.halfExtents[2] * absR[i][2];

		if (std::abs(trans[i]) > ra + rb)
			return false;

		depth = std::abs(rb - std::abs((ra - std::abs(trans[i]))));

		if (depth < contactInfo.depth)
		{
			contactInfo.depth = depth;
			contactInfo.normal = obb2.localAxes[i];
		}
	}

	//Test B0, B1, B2
	for (int i = 0; i < 3; ++i)
	{
		ra = obb1.halfExtents[0] * absR[0][i] + obb1.halfExtents[1] * absR[1][i] + obb1.halfExtents[2] * absR[2][i];
		rb = obb2.halfExtents[i]; 

		if (std::abs(trans[0] * R[0][i] + trans[1] * R[1][i] + trans[2] * R[2][i]) > ra + rb)
			return false;

		depth = std::abs(rb - std::abs((ra - std::abs(trans[i]))));

		if (depth < contactInfo.depth)
		{
			contactInfo.depth = depth;
			contactInfo.normal = -obb1.localAxes[i];
		}
	}

	//Test A0 x B0
	ra = obb1.halfExtents[1] * absR[2][0] + obb1.halfExtents[2] * absR[1][0];
	rb = obb2.halfExtents[1] * absR[0][2] + obb2.halfExtents[2] * absR[0][1];
	if (std::abs(trans[2] * R[1][0] - trans[1] * R[2][0]) > ra + rb)
		return false;

	//Test A0 x B1
	ra = obb1.halfExtents[1] * absR[2][1] + obb1.halfExtents[2] * absR[1][1];
	rb = obb2.halfExtents[0] * absR[0][2] + obb2.halfExtents[2] * absR[0][0];
	if (std::abs(trans[2] * R[1][1] - trans[1] * R[2][1]) > ra + rb)
		return false;

	//Test A0 x B2
	ra = obb1.halfExtents[1] * absR[2][2] + obb1.halfExtents[2] * absR[1][2];
	rb = obb2.halfExtents[0] * absR[0][1] + obb2.halfExtents[1] * absR[0][0];
	if (std::abs(trans[2] * R[1][2] - trans[1] * R[2][2]) > ra + rb)
		return false;

	//Test A1 x B0
	ra = obb1.halfExtents[0] * absR[2][0] + obb1.halfExtents[2] * absR[0][0];
	rb = obb2.halfExtents[1] * absR[1][2] + obb2.halfExtents[2] * absR[1][1];
	if (std::abs(trans[0] * R[2][0] - trans[2] * R[0][0]) > ra + rb)
		return false;

	//Test A1 x B1
	ra = obb1.halfExtents[0] * absR[2][1] + obb1.halfExtents[2] * absR[0][1];
	rb = obb2.halfExtents[0] * absR[1][2] + obb2.halfExtents[2] * absR[1][0];
	if (std::abs(trans[0] * R[2][1] - trans[2] * R[0][1]) > ra + rb)
		return false;

	//Test A1 x B2
	ra = obb1.halfExtents[0] * absR[2][2] + obb1.halfExtents[2] * absR[0][2];
	rb = obb2.halfExtents[0] * absR[1][1] + obb2.halfExtents[2] * absR[1][0];
	if (std::abs(trans[0] * R[2][2] - trans[2] * R[0][2]) > ra + rb)
		return false;

	//Test A2 x B0
	ra = obb1.halfExtents[0] * absR[1][0] + obb1.halfExtents[1] * absR[0][0];
	rb = obb2.halfExtents[1] * absR[2][2] + obb2.halfExtents[2] * absR[2][1];
	if (std::abs(trans[1] * R[0][0] - trans[0] * R[1][0]) > ra + rb)
		return false;

	//Test A2 x B1
	ra = obb1.halfExtents[0] * absR[1][1] + obb1.halfExtents[1] * absR[0][1];
	rb = obb2.halfExtents[0] * absR[2][2] + obb2.halfExtents[2] * absR[2][0];
	if (std::abs(trans[1] * R[0][1] - trans[0] * R[1][1]) > ra + rb)
		return false;

	//Test A2 x B2
	ra = obb1.halfExtents[0] * absR[1][2] + obb1.halfExtents[1] * absR[0][2];
	rb = obb2.halfExtents[0] * absR[2][1] + obb2.halfExtents[1] * absR[2][0];
	if (std::abs(trans[1] * R[0][2] - trans[0] * R[1][2]) > ra + rb)
		return false;

	return true;
}
