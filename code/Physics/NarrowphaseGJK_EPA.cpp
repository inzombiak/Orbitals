#include "NarrowphaseGJK_EPA.h"
#include "glm\gtc\matrix_transform.hpp"
#include "glm\gtx\norm.hpp"

void NarrowphaseGJK_EPA::PerformCollisionResolution(const std::vector<PhysicsDefs::CollisionPair>& collisionPairs)
{
	PhysicsDefs::Contact contactData;
	PhysicsDefs::AABB aabb1, aabb2;
	glm::vec3 vel1, vel2, relativeVel, impulse, correction;
	glm::mat4 invTransform1;
	glm::mat4& interpolationTrans1 = glm::mat4(1);
	glm::mat4& interpolationTrans2 = glm::mat4(1);
	float velAlongColNormal, minConst, totalSytemMass, mass1, mass2, invMass1, invMass2;

	for (int i = 0; i < collisionPairs.size(); ++i)
	{
		interpolationTrans1 = collisionPairs[i].first->GetInterpolationTransform();
		interpolationTrans2 = collisionPairs[i].second->GetInterpolationTransform();
		if (RunGJK_EPA(collisionPairs[i].first, collisionPairs[i].second, contactData))
		{
			vel1 = collisionPairs[i].first->GetLinearVelocity();
			vel2 = collisionPairs[i].second->GetLinearVelocity();
			relativeVel = vel2 - vel1;
			velAlongColNormal = glm::dot(relativeVel, contactData.normal);

			if (velAlongColNormal < 0)
				continue;

			minConst = std::min(collisionPairs[i].first->GetRestitution(), collisionPairs[i].second->GetRestitution());
			mass1 = collisionPairs[i].first->GetMass();
			invMass1 = collisionPairs[i].first->GetInverseMass();
			mass2 = collisionPairs[i].second->GetMass();
			invMass2 = collisionPairs[i].second->GetInverseMass();
			totalSytemMass = mass1 + mass2;

			impulse = contactData.normal * (-(1 + minConst) * velAlongColNormal);

			collisionPairs[i].first->ApplyImpulse(-impulse * mass1 / totalSytemMass);
			collisionPairs[i].second->ApplyImpulse(impulse * mass2 / totalSytemMass);

			//Pushout to avoid sinking
			const float PERCENT = 1;
			const float THRESHOLD = 0.001;
			correction = std::max(contactData.depth - THRESHOLD, 0.0f) / (invMass1 + invMass2) * PERCENT * contactData.normal;
			interpolationTrans1 = glm::translate(interpolationTrans1, invMass1 * correction);
			interpolationTrans2 = glm::translate(interpolationTrans2, -invMass2 * correction);
			collisionPairs[i].first->UpdateInterpolationTransform(interpolationTrans1);
			collisionPairs[i].second->UpdateInterpolationTransform(interpolationTrans2);
			//collisionPairs[i].first->SetLinearVelocity(glm::vec3(0, 0, 0));
			//collisionPairs[i].second->SetLinearVelocity(glm::vec3(0, 0, 0));
		}
	}
	
}

bool NarrowphaseGJK_EPA::RunGJK_EPA(IRigidBody* bodyA, IRigidBody* bodyB, PhysicsDefs::Contact& contactData)
{
	glm::vec3 dir;
	glm::mat4 bodyBtoATrans = glm::inverse(bodyA->GetInterpolationTransform()) * bodyB->GetInterpolationTransform();
	PhysicsDefs::SupportPoint nextSupportPoint;
	m_faceList.clear();

	std::vector<PhysicsDefs::SupportPoint> simplex;

	//Init simplex
	dir = glm::vec3(1, 0, 0);

	nextSupportPoint.originA = bodyA->GetSupportPoint(dir);
	nextSupportPoint.originB = glm::vec3(bodyBtoATrans * glm::vec4(bodyB->GetSupportPoint(-dir), 1));
	nextSupportPoint.position = nextSupportPoint.originA - nextSupportPoint.originB;
	nextSupportPoint.dir = dir;

	simplex.push_back(nextSupportPoint);

	dir = dir * -1.f;

	bool intersection = false;

	while (true)
	{
		nextSupportPoint.originA = bodyA->GetSupportPoint(dir);
		nextSupportPoint.originB = glm::vec3(bodyBtoATrans * glm::vec4(bodyB->GetSupportPoint(-dir), 1));
		nextSupportPoint.position = nextSupportPoint.originA - nextSupportPoint.originB;
		nextSupportPoint.dir = dir;

		//No intersection
		if (glm::dot(nextSupportPoint.position, dir) < 0)
		{
			intersection = false;
			break;
		}
		simplex.push_back(nextSupportPoint);
		if (DoSimplex(simplex, dir))
		{
			intersection = true;
			break;
		}

		dir = glm::normalize(dir);
	}

	//Don't run epa
	if (!intersection)
		return false;

	contactData = GetContactInfo(bodyA, bodyB, simplex);

	return true;
}

PhysicsDefs::Contact NarrowphaseGJK_EPA::GetContactInfo(IRigidBody* bodyA, IRigidBody* bodyB, std::vector<PhysicsDefs::SupportPoint>& simplex)
{
	//This should never happen, but I'm leaving it just in case
	if (simplex.size() < 4)
		assert(false);
	PhysicsDefs::Contact result;
	PhysicsDefs::SupportPoint nextSupportPoint;
	FaceListIterator closestFaceIt;
	glm::mat4 bodyBtoATrans = glm::inverse(bodyA->GetInterpolationTransform()) * bodyB->GetInterpolationTransform();
	//Construct initial faces
	m_faceList.emplace_back(Face(simplex[3], simplex[1], simplex[2]));
	m_faceList.emplace_back(Face(simplex[3], simplex[2], simplex[0]));
	m_faceList.emplace_back(Face(simplex[3], simplex[0], simplex[1]));
	m_faceList.emplace_back(Face(simplex[2], simplex[1], simplex[0]));

	//float lastDist = FLT_MAX;;

	while (true)
	{
		closestFaceIt = FindClosestFace();

		//Get next point
		nextSupportPoint.originA = bodyA->GetSupportPoint(closestFaceIt->normal);
		nextSupportPoint.originB = glm::vec3(bodyBtoATrans * glm::vec4(bodyB->GetSupportPoint(-closestFaceIt->normal), 1));
		nextSupportPoint.position = nextSupportPoint.originA - nextSupportPoint.originB;
		nextSupportPoint.dir = closestFaceIt->normal;

		if (1 - glm::length2(closestFaceIt->normal) > 0.0001f)
		{
			assert(0);
		}
		auto dist = std::abs(glm::dot(closestFaceIt->normal, nextSupportPoint.position)) - closestFaceIt->distance;
		if (dist  < 0.001f)
		{
			result.normal = -closestFaceIt->normal;
			result.depth = closestFaceIt->distance;
			break;
		}

		//Remove all visible faces
		FaceListIterator it = m_faceList.begin();
		while (it != m_faceList.end())
		{
			if (glm::dot(it->normal, nextSupportPoint.position - it->a.position) > 0)
			{
				AddFaceEdges(it);
				it = m_faceList.erase(it);
			}
			else
			{
				++it;
			}
		}

		EdgeListIterator edgeIt = m_edgeList.begin();
		while (edgeIt != m_edgeList.end())
		{
			m_faceList.emplace_back(Face(nextSupportPoint, edgeIt->a, edgeIt->b));
			++edgeIt;
		}
		m_edgeList.clear();
	}

	//Calculate barycentric coordinates
	Face f = *closestFaceIt;

	//Project origin onto normal
	glm::vec3 originProjNormal = glm::dot(-f.a.position, f.normal) * f.normal;// / glm::length2(f.normal);
	glm::vec3 originProjPlane = -f.a.position - originProjNormal;

	//From Real-Time Collision Detection by Crister Erickson
	glm::vec3 v0 = f.b.position - f.a.position, v1 = f.c.position - f.a.position, v2 = originProjPlane-f.a.position;
	float d00 = glm::dot(v0, v0);
	float d01 = glm::dot(v0, v1);
	float d11 = glm::dot(v1, v1);
	float d20 = glm::dot(v2, v0);
	float d21 = glm::dot(v2, v1);
	float denom = d00 * d11 - d01 * d01;
	float v = (d11 * d20 - d01 * d21) / denom;
	float w = (d00 * d21 - d01 * d20) / denom;
	float u = 1.0f - v - w;

	result.worldPointA = u * f.a.originA + v * f.b.originA + w* f.c.originA;
	result.worldPointB = u * f.a.originB + v * f.b.originB + w* f.c.originB;
	float test = glm::length(result.worldPointB - result.worldPointA);
	return result;
}

//For explanation, see here http://hacktank.net/blog/?p=119
void NarrowphaseGJK_EPA::AddFaceEdges(NarrowphaseGJK_EPA::FaceListIterator it)
{
	Edge ab, bc, ca;
	ab.a = it->a;
	ab.b = it->b;
	InsertEdge(ab);

	bc.a = it->b;
	bc.b = it->c;
	InsertEdge(bc);

	ca.a = it->c;
	ca.b = it->a;
	InsertEdge(ca);

}

void NarrowphaseGJK_EPA::InsertEdge(const Edge& edge)
{
	for (EdgeListIterator it = m_edgeList.begin(); it != m_edgeList.end(); ++it)
	{
		if (it->b.position == edge.a.position && it->a.position == edge.b.position)
		{
			it = m_edgeList.erase(it);
			return;
		}
	}

	m_edgeList.emplace_back(edge);
}

NarrowphaseGJK_EPA::FaceListIterator NarrowphaseGJK_EPA::FindClosestFace()
{
	FaceListIterator result;
	float minDist = FLT_MAX;
	glm::vec3 norm, edge;

	for (FaceListIterator it = m_faceList.begin(); it != m_faceList.end(); ++it)
	{
		if (it->distance < minDist)
		{
			minDist = it->distance;
			result = it;
		}
	}

	return result;
}

bool NarrowphaseGJK_EPA::DoSimplex(std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir)
{
	/*

	This function makes use of helpers to detect collision
	The process can be sped up by not used the functions and instead writing out each case by hand
	*/

	glm::vec3 newDir;
	if (simplex.size() == 1)
	{
		//Vertex
		dir = -simplex[0].position;
	}
	else if (simplex.size() == 2)
	{
		//Edge
		DoSimplexEdge(1, 0, simplex, dir);
	}
	else if (simplex.size() == 3)
	{
		DoSimplexTriangle(2, 1, 0, simplex, dir);
	}
	else if (simplex.size() == 4)
	{
		//Tetrahedron
		return DoSimplexTetrahedron(simplex, dir);
	}

	return false;
}

bool NarrowphaseGJK_EPA::DoSimplexEdge(int aIndex, int bIndex, std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir)
{
	PhysicsDefs::SupportPoint a = simplex[aIndex];
	PhysicsDefs::SupportPoint b = simplex[bIndex];
	glm::vec3 ab = b.position - a.position;
	std::vector<PhysicsDefs::SupportPoint> newSimplex;
	if (glm::dot(ab, -a.position) > 0)
	{
		dir = glm::cross(glm::cross(ab, -a.position), ab);

		newSimplex.push_back(b);
		newSimplex.push_back(a);
		simplex = newSimplex;

		return true;
	}
	else
	{
		dir = -a.position;
		newSimplex.push_back(a);
		simplex = newSimplex;

		return false;
	}
}

bool NarrowphaseGJK_EPA::DoSimplexTriangle(int aIndex, int bIndex, int cIndex, std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir)
{
	//Triangle
	PhysicsDefs::SupportPoint a = simplex[aIndex];
	PhysicsDefs::SupportPoint b = simplex[bIndex];
	PhysicsDefs::SupportPoint c = simplex[cIndex];
	glm::vec3 ac = c.position - a.position;
	glm::vec3 ab = b.position - a.position;
	glm::vec3 abc = glm::cross(ab, ac);
	std::vector<PhysicsDefs::SupportPoint> newSimplex;

	//If AC is closest to origin
	if (glm::dot(glm::cross(abc, ac), -a.position) > 0)
	{
		if (glm::dot(ac, -a.position) > 0)
		{
			dir = glm::cross(glm::cross(ac, -a.position), ac);
			newSimplex.push_back(c);
			newSimplex.push_back(a);

			simplex = newSimplex;
		}
		else
		{
			DoSimplexEdge(aIndex, bIndex, simplex, dir);
		}
	}
	else
	{
		//If AB is closest to origin
		if (glm::dot(glm::cross(ab, abc), -a.position) > 0)
		{
			DoSimplexEdge(aIndex, bIndex, simplex, dir);
		}
		else if (glm::dot(abc, -a.position) > 0)
		{
			//Above Triangle
			dir = abc;
			newSimplex.push_back(c);
			newSimplex.push_back(b);
			newSimplex.push_back(a);
			simplex = newSimplex;
		}
		else
		{
			//Underneath Triangle
			dir = -abc;

			//Change vertex order to search down
			newSimplex.push_back(b);
			newSimplex.push_back(c);
			newSimplex.push_back(a);
			simplex = newSimplex;
		}
	}

	return false;
}



bool NarrowphaseGJK_EPA::DoSimplexTetrahedron(std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir)
{
	//Triangle
	PhysicsDefs::SupportPoint a = simplex[3];
	PhysicsDefs::SupportPoint b = simplex[2];
	PhysicsDefs::SupportPoint c = simplex[1];
	PhysicsDefs::SupportPoint d = simplex[0];
	glm::vec3 ac = c.position - a.position;
	glm::vec3 ab = b.position - a.position;
	glm::vec3 ad = d.position - a.position;
	glm::vec3 abc = glm::cross(ab, ac);
	glm::vec3 adb = glm::cross(ad, ab);
	glm::vec3 acd = glm::cross(ac, ad);
	std::vector<PhysicsDefs::SupportPoint> newSimplex;

	//Check ABC
	if (glm::dot(abc, -a.position) > 0)
		DoSimplexTriangle(3, 2, 1, simplex, dir);
	else if (glm::dot(adb, -a.position) > 0)
		DoSimplexTriangle(3, 0, 2, simplex, dir);
	else if (glm::dot(acd, -a.position) > 0)
		DoSimplexTriangle(3, 1, 0, simplex, dir);
	else
		return true;

	return false;
}