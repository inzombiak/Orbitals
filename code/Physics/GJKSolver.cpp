#include "GJKSolver.h"

bool GJKSolver::CheckCollision(const std::vector<glm::vec3>& shapeA, const std::vector<glm::vec3>& shapeB, PhysicsDefs::Contact& contactData)
{
	glm::vec3 dir;
	PhysicsDefs::SupportPoint nextSupportPoint;
	m_faceList.clear();

	std::vector<PhysicsDefs::SupportPoint> simplex;

	//Init simplex
	dir = shapeA[0];
	nextSupportPoint = PhysicsDefs::GetSupportPoint(shapeA, shapeB, dir);
	simplex.push_back(nextSupportPoint);


	dir = dir * -1.f;

	bool intersection = false;

	while (true)
	{
		nextSupportPoint = PhysicsDefs::GetSupportPoint(shapeA, shapeB, dir);
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
	}

	//Don't run epa
	if (!intersection)
		return intersection;

	contactData = GetContactInfo(shapeA, shapeB, simplex);

	//simplexOut = simplex;
	return intersection;
}

PhysicsDefs::Contact GJKSolver::GetContactInfo(const std::vector<glm::vec3>& shapeA, const std::vector<glm::vec3>& shapeB, std::vector<PhysicsDefs::SupportPoint>& simplex)
{
	//This should never happen, but I'm leaving it just in case
	if (simplex.size() < 4)
		assert(false);

	PhysicsDefs::Contact result;
	PhysicsDefs::SupportPoint nextSupportPoint;
	Face f;
	FaceListIterator closestFaceIt;
	//Construct initial faces
	f.a = simplex[0];
	f.b = simplex[2];
	f.c = simplex[1];
	f.CalculateNormal();
	f.CalculateDistanceToOrigin();
	m_faceList.push_back(f);

	f.a = simplex[0];
	f.b = simplex[3];
	f.c = simplex[2];
	f.CalculateNormal();
	f.CalculateDistanceToOrigin();
	m_faceList.push_back(f);

	f.a = simplex[0];
	f.b = simplex[1];
	f.c = simplex[3];
	f.CalculateNormal();
	f.CalculateDistanceToOrigin();
	m_faceList.push_back(f);
	
	f.a = simplex[1];
	f.b = simplex[2];
	f.c = simplex[3];
	f.CalculateNormal();
	f.CalculateDistanceToOrigin();
	m_faceList.push_back(f);

	float lastDist = FLT_MAX;;

	while (true)
	{
		closestFaceIt = FindClosestFace();

		//If we didnt make much progress, terminate
		if (lastDist - closestFaceIt->distance < 0.0001f)
		{
			result.normal = closestFaceIt->normal;
			result.depth = closestFaceIt->distance;
			break;
		}
		
		//Get next point
		nextSupportPoint = PhysicsDefs::GetSupportPoint(shapeA, shapeB, closestFaceIt->normal);
		
		//Remove all visible faces
		FaceListIterator it = m_faceList.begin();
		while(it != m_faceList.end())
		{
			if (glm::dot(closestFaceIt->normal, nextSupportPoint.position - closestFaceIt->a.position) > 0)
			{
				AddFaceEdges(it);
				m_faceList.erase(it);
			}
			else
			{
				++it;
			}
		}



	}

	//Calculate barycentric coordinates
	f = *closestFaceIt;

	//From Real-Time Collision Detection by Crister Erickson
	glm::vec3 v0 = f.b.position - f.a.position, v1 = f.c.position - f.a.position, v2 = -f.a.position;
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

	return result;
}

void GJKSolver::AddFaceEdges(GJKSolver::FaceListIterator it)
{
	Edge ab, bc, ac;
}

GJKSolver::FaceListIterator GJKSolver::FindClosestFace()
{
	FaceListIterator result;
	float minDist = FLT_MAX, currDist;
	glm::vec3 norm, edge;

	for (FaceListIterator it = m_faceList.begin(); it != m_faceList.end(); ++it)
	{
		if (it->distance < minDist)
		{
			minDist = currDist;
			result = it;
		}
	}

	return result;
}

bool GJKSolver::DoSimplex(std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir)
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
	else if(simplex.size() == 3)
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

bool GJKSolver::DoSimplexEdge(int aIndex, int bIndex, std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir)
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

bool GJKSolver::DoSimplexTriangle(int aIndex, int bIndex, int cIndex, std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir)
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



bool GJKSolver::DoSimplexTetrahedron(std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir)
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
	glm::vec3 abd = glm::cross(ad, ab);
	glm::vec3 acd = glm::cross(ac, ad);
	std::vector<PhysicsDefs::SupportPoint> newSimplex;

	//Check ABC
	if (glm::dot(abc, -a.position) > 0)
		DoSimplexTriangle(3, 2, 1, simplex, dir);
	else if (glm::dot(abd, -a.position) > 0)
		DoSimplexTriangle(3, 2, 0, simplex, dir);
	else if (glm::dot(acd, -a.position) > 0)
		DoSimplexTriangle(3, 1, 0, simplex, dir);
	else
		return true;

	return false;
}

std::list<GJKSolver::Face> GJKSolver::m_faceList;
std::list<GJKSolver::Edge> GJKSolver::m_edgeList;