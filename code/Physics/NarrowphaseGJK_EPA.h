#ifndef NARROWPHASE_GJK_EPA_H
#define NARROWPHASE_GJK_EPA_H

#include "INarrowphase.h"

#include <list>

class NarrowphaseGJK_EPA : public INarrowphase
{
public:
	void PerformCollisionResolution(const std::vector<PhysicsDefs::CollisionPair>& collisionPairs, ErrorCallBack ecb = 0);

private:
	bool RunGJK_EPA(IRigidBody* bodyA, IRigidBody* bodyB, PhysicsDefs::Contact& contactData, ErrorCallBack ecb = 0);

	//EPA
	struct Face
	{
		PhysicsDefs::SupportPoint a;
		PhysicsDefs::SupportPoint b;
		PhysicsDefs::SupportPoint c;

		Face(PhysicsDefs::SupportPoint A, PhysicsDefs::SupportPoint B, PhysicsDefs::SupportPoint C)
		{
			a = A;
			b = B;
			c = C;

			CalculateNormal();

			////Normal should point away from origin
			//if (glm::dot(normal, a.position) < 0)
			//{
			//	std::swap(b, c);
			//	CalculateNormal();
			//	if (glm::dot(normal, a.position) < 0)
			//		assert(0);
			//}

			CalculateDistanceToOrigin();
		}

		glm::vec3 normal;
		float distance;
		void CalculateNormal()
		{
			glm::vec3 ab = b.position - a.position, ac = c.position - a.position;
			normal = glm::normalize(glm::cross(ab, ac));
		}
		void CalculateDistanceToOrigin()
		{
			//TODO: WHAT IF 2 TRIANGLES SHARE A AND HAVE THE SAME NORMAL????
			distance = std::fabs(glm::dot(normal, a.position));
			//Taken from Real-time Collision Detection by Christer Ericson
		}
	};

	struct Edge
	{
		PhysicsDefs::SupportPoint a;
		PhysicsDefs::SupportPoint b;
	};

	//Ewwwwwwwwwww
	//TODO: Lists are bad, find a better way to deal with insertions and removales
	typedef std::list<Face>::iterator FaceListIterator;
	std::list<Face> m_faceList;
	typedef std::list<Edge>::iterator EdgeListIterator;
	std::list<Edge> m_edgeList;

	PhysicsDefs::Contact GetContactInfo(IRigidBody* bodyA, IRigidBody* bodyB, std::vector<PhysicsDefs::SupportPoint>& simplex);
	FaceListIterator FindClosestFace();
	void AddFaceEdges(FaceListIterator face);
	void InsertEdge(const Edge& edge);

	//GJK
	bool DoSimplex(std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir);
	bool DoSimplexEdge(int aIndex, int bIndex, std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir);
	bool DoSimplexTriangle(int aIndex, int bIndex, int cIndex, std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir);
	bool DoSimplexTetrahedron(std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir);
};

#endif