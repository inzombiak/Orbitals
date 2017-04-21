#ifndef GJK_SOLVER_H
#define GJK_SOLVER_H

#include "IRigidBody.h"
#include <list>

class GJKSolver
{

public:
	static bool CheckCollision(const std::vector<glm::vec3>& shapeA, const std::vector<glm::vec3>& shapeB, PhysicsDefs::Contact& contactData);


private:
	

	//EPA
	struct Face
	{
		PhysicsDefs::SupportPoint a;
		PhysicsDefs::SupportPoint b;
		PhysicsDefs::SupportPoint c;
		glm::vec3 normal;
		float distance;
		void CalculateNormal()
		{
			normal = glm::normalize(glm::cross(b.position - a.position, c.position - a.position));
		}
		void CalculateDistanceToOrigin()
		{
			//TODO: WHAT IF 2 TRIANGLES SHARE A AND HAVE THE SAME NORMAL????
			distance = glm::dot(normal, a.position);
			//Taken from Real-time Collision Detection by Christer Ericson
		}
	};

	struct Edge
	{
		glm::vec3 a;
		glm::vec3 b;
	};
	
	//Ewwwwwwwwwww
	//TODO: Lists are bad, find a better way to deal with insertions and removales
	typedef std::list<Face>::iterator FaceListIterator;
	static std::list<Face> m_faceList;
	typedef std::list<Face>::iterator EdgeListIterator;
	static std::list<Edge> m_edgeList;

	static PhysicsDefs::Contact GetContactInfo(const std::vector<glm::vec3>& shapeA, const std::vector<glm::vec3>& shapeB, std::vector<PhysicsDefs::SupportPoint>& simplex);
	static FaceListIterator FindClosestFace();
	static void AddFaceEdges(FaceListIterator face);
	//GJK
	static bool DoSimplex(std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir);
	static bool DoSimplexEdge(int aIndex, int bIndex, std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir);
	static bool DoSimplexTriangle(int aIndex, int bIndex, int cIndex, std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir);
	static bool DoSimplexTetrahedron(std::vector<PhysicsDefs::SupportPoint>& simplex, glm::vec3& dir);

	

};

#endif