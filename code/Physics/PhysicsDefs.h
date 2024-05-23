#ifndef PHYSICS_DEFS_H
#define PHYSICS_DEFS_H

#include <array>
#include <vector>
#define _USE_MATH_DEFINES
#include <math.h> 

#include "../Utilities/GameDefs.h"

#include <glm/gtc/quaternion.hpp>

#include "OTransform.h"
class ICelestialObject;
class ICollisionShape;
class IRigidBody;
namespace PhysicsDefs
{
	enum RigidBodyType
	{
		Static = 1,
		Dynamic = 2,
	};
	enum CollisionShapeType
	{
		Box = 1,
		Sphere = 2,
	};
	enum PhysicsBodyType
	{
		RigidBody= 1,
		CompoundBody = 2,
		NULLBody = 4,
	};

	struct ContactInfo
	{
		//Contact point in global space
		glm::vec3 worldPos;

		//Contact points in global space
		glm::vec3 worldPointA;
		glm::vec3 worldPointB;

		//Contact points in local space
		glm::vec3 localPointA;
		glm::vec3 localPointB;

		//Contact normal and tangents
		glm::vec3 normal;
		glm::vec3 tangent1, tangent2;

		//Depth of penetration
		float depth;

		//TODO: Maybe move it out
		float prevNormalImp = 0.f;
		float prevTangImp1 = 0.f;
		float prevTangImp2 = 0.f;

		//Calculated in prestep
		float massNormal;
		float massTangent1;
		float massTangent2;
		float bias;
		float friction;

	};

	struct RigidBodyConstructionInfo
	{
		float mass = 0;
		float linearDamping = 0;
		float angularDamping = 0;
		float friction = 0;
		float rollingFriction = 0;
		float resititution = 0;

		OTransform transform;
		glm::vec3 localInertia = glm::vec3(0, 0, 0);

		bool enableGravity = true;

		ICollisionShape* collisionShape = 0;
	};

	struct SupportPoint
	{
		glm::vec3 position = glm::vec3(0, 0, 0);
		glm::vec3 originA = glm::vec3(0, 0, 0);
		glm::vec3 originB = glm::vec3(0, 0, 0);
		glm::vec3 dir = glm::vec3(0, 0, 0);
	};

	struct AABB
	{
		glm::vec3 min = glm::vec3(0, 0, 0);
		glm::vec3 max = glm::vec3(0, 0, 0);

		glm::mat4 worldTransform;
		IRigidBody* body = 0;

		bool Intersects(const AABB& other) const
		{
			if (max.x <= other.min.x || min.x >= other.max.x)
				return false;
			if (max.y <= other.min.y || min.y >= other.max.y)
				return false;
			if (max.z <= other.min.z || min.z >= other.max.z)
				return false;

			return true;
		}
	};

	struct OBB
	{
		glm::vec3 pos;
		glm::mat3 localAxes;
		glm::vec3 halfExtents;

		PhysicsDefs::AABB aabb;

		//TODO: SPEED THIS UP
		void UpdateAABB()
		{
			static const glm::vec3 OFFSET(0.01f);
			aabb.min = glm::vec3(FLT_MAX);
			aabb.max = glm::vec3(FLT_MIN);
			glm::vec3 x, y, z;
			
			x = (halfExtents.x + OFFSET.x) * localAxes[0];
			y = (halfExtents.y + OFFSET.y) * localAxes[1];
			z = (halfExtents.z + OFFSET.z) * localAxes[2];
			
			/*aabb.min = -(x + y + z);
			aabb.max = (x + y + z);*/
			glm::vec3 currentVertex, currMultipliers(-1, -1, -1);
			for (int i = 0; i < 8; ++i)
			{
				if (i == 4)
					currMultipliers.y *= -1;

				currentVertex = x*currMultipliers.x + y*currMultipliers.y + z*currMultipliers.z;


				if (aabb.min.x > currentVertex.x)
					aabb.min.x = currentVertex.x;
				if (aabb.max.x < currentVertex.x)
					aabb.max.x = currentVertex.x;

				if (aabb.min.y > currentVertex.y)
					aabb.min.y = currentVertex.y;
				if (aabb.max.y < currentVertex.y)
					aabb.max.y = currentVertex.y;

				if (aabb.min.z > currentVertex.z)
					aabb.min.z = currentVertex.z;
				if (aabb.max.z < currentVertex.z)
					aabb.max.z = currentVertex.z;

				if (i % 2 == 0)
					currMultipliers.x *= -1;
				else
					currMultipliers.z *= -1;

			}

		}
	};

	typedef std::pair<IRigidBody*, IRigidBody*> CollisionPair;
	typedef std::pair<CollisionPair, ContactInfo> CollPairContactInfo;
	
	class ICreationData
	{
	public:
		virtual ~ICreationData() {};

		CollisionShapeType GetShape()
		{
			return shapeType;
		}

		PhysicsBodyType GetBody()
		{
			return bodyType;
		}

		int id = -1;
		ICelestialObject* owner = 0;
		virtual ICreationData* clone() const = 0;
	protected:
		virtual void SetBodyAndShape() {};
		CollisionShapeType shapeType = CollisionShapeType::Box;
		PhysicsBodyType bodyType = PhysicsBodyType::NULLBody;

	};
	class IRigidBodyCreationData : public ICreationData
	{
	public:
		virtual ~IRigidBodyCreationData() {};
		RigidBodyConstructionInfo rbci;
	};
	class BoxCreationData : public IRigidBodyCreationData
	{
	public:
		BoxCreationData()
		{
			SetBodyAndShape();
		}

		virtual ~BoxCreationData() {};
		glm::vec3 boxDimensions;
	
		BoxCreationData* clone() const
		{
			return new BoxCreationData(*this);
		}

	protected:
		void SetBodyAndShape() override
		{
			ICreationData::shapeType = CollisionShapeType::Box;
			ICreationData::bodyType = PhysicsBodyType::RigidBody;
		}
	};
	class SphereCreationData : public IRigidBodyCreationData
	{
	public:
		SphereCreationData()
		{
			SetBodyAndShape();
		}
		virtual ~SphereCreationData() {};
		float sphereRadius;

		SphereCreationData* clone() const
		{
			return new SphereCreationData(*this);
		}

	protected:
		void SetBodyAndShape() override
		{
			ICreationData::shapeType = CollisionShapeType::Sphere;
			ICreationData::bodyType = PhysicsBodyType::RigidBody;
		}
	};

	struct Edge
	{
		Edge(const glm::vec2& s, const glm::vec2 e)
		{
			start = s;
			end = e;
			normal.x = -e.y + s.y;
			normal.y = e.x - s.x;
		}

		bool IsInFront(glm::vec2 point)
		{
			return glm::dot(normal, point - start) < 0;
		}

		glm::vec2 start, end, normal;
	};
	
	struct Ray
	{
		Ray()
		{}
		Ray(const glm::vec2& o, const glm::vec2& d)
		{
			origin = o;
			direction = d;
		}

		glm::vec2 origin = glm::vec2(0, 0);
		glm::vec2 direction = glm::vec2(1, 1);
	};
	
	inline float Cross2D(const glm::vec2& a, const glm::vec2& b)
	{
		return a.x * b.y - b.x * a.y;
	}

	inline bool InfLineInfLineIntersect(const Ray& rayA, const Ray& rayB, glm::vec2& intersectPoint)
	{
		//Cross product of the directions
		float dirCross = Cross2D(rayA.direction, rayB.direction);
		if (dirCross == 0)
			return false;

		glm::vec2 originDiff = rayB.origin - rayA.origin;
		float t = (Cross2D(originDiff, rayB.direction)) / dirCross;
		float u = (Cross2D(originDiff, rayA.direction)) / dirCross;

		t = roundf(t * 100) / 100;
		intersectPoint = rayA.origin + (float)t * rayA.direction;
		return true;
	}


	inline std::vector<glm::vec2> ClipPolygon(const std::vector<glm::vec2>& polygon, const std::vector<glm::vec2>& clippingPlane)
	{
		std::vector<glm::vec2> currShape = polygon;
		std::vector<glm::vec2> result;
		glm::vec2 s, e, intersectionPoint;

		for (int i = 0; i < clippingPlane.size() - 1; ++i)
		{
			result.resize(0);
			
			//TODO:: ??????
			if (currShape.size() == 0)
				return result;

			Edge clipEdge(clippingPlane[i], clippingPlane[i + 1]);
			s = currShape[currShape.size() - 1];

			bool test = false;

			for (int j = 0; j < currShape.size(); ++j)
			{
				e = currShape[j];
				//IsOnRight = true means point is "inside" plane
				if (clipEdge.IsInFront(s))
				{
					if (!clipEdge.IsInFront(e))
					{
						/*if (sfmath::RayLineIntersect(sfmath::Ray(clipEdge.start, clipEdge.end- clipEdge.start), s, e, intersectionPoint))
						result.push_back(intersectionPoint);*/
						if (InfLineInfLineIntersect(Ray(s, e - s), Ray(clipEdge.start, clipEdge.end - clipEdge.start), intersectionPoint))
							result.push_back(intersectionPoint);
					}
					else
						result.push_back(e);
					test = true;
				}
				else if (clipEdge.IsInFront(e))
				{
					if (InfLineInfLineIntersect(Ray(s, e - s), Ray(clipEdge.start, clipEdge.end - clipEdge.start), intersectionPoint))
						result.push_back(intersectionPoint);
					result.push_back(e);
					test = true;
				}
				s = e;

				if (!test)
					printf("TEST FLASE \n");

			}
			if (result.size() < 4)
				printf("TE2ST FLASE \n");
			currShape = result;
		}

		//result.clear();
		//Edge clipEdge(clippingPlane.back(), clippingPlane[0]);
		//s = currShape.back();
		//for (int j = 0; j < currShape.size(); ++j)
		//{
		//	e = currShape[j];
		//	//IsOnRight = true means point is "inside" plane
		//	if (clipEdge.IsInFront(s))
		//	{
		//		if (!clipEdge.IsInFront(e))
		//		{
		//			if (InfLineInfLineIntersect(Ray(s, e - s), Ray(clipEdge.start, clipEdge.end - clipEdge.start), intersectionPoint))
		//				result.push_back(intersectionPoint);
		//		}
		//		else
		//			result.push_back(e);
		//	}
		//	else if (clipEdge.IsInFront(e))
		//	{
		//		if (InfLineInfLineIntersect(Ray(s, e - s), Ray(clipEdge.start, clipEdge.end - clipEdge.start), intersectionPoint))
		//			result.push_back(intersectionPoint);
		//		result.push_back(e);
		//	}
		//	s = e;
		//}
		return result;
	}

	inline float Clamp(float n, float min, float max)
	{
		if (n < min)
			return min;
		if (n > max)
			return max;

		return n;
	}


	inline void RayClosestApproach(const glm::vec3& pointA, const glm::vec3& dirA,
								   const glm::vec3& pointB, const glm::vec3& dirB,
								   float& alpha, float& beta)
	{
		glm::vec3 r = pointA - pointB;

		//Squared lengths
		float ab = glm::dot(dirA, dirB);
		float q1 = glm::dot(dirA, r);
		float q2 = -glm::dot(dirB, r);
		float d = 1 - ab - ab;
		if (d <= -.0001)
		{
			alpha = 0;
			beta = 0;
		}
		else
		{
			d = 1.f / d;
			alpha = (q1 + ab * q2) * d;
			beta = (ab * q1 + q2) * d;
		}

	}

	inline float DistanceToLineSq(const glm::vec3& start, const glm::vec3& end, const glm::vec3& p)
	{
		glm::vec3 pStart = p - start;
		glm::vec3 line = end - start;
		glm::vec3 proj = line * glm::dot(pStart, line) / glm::dot(line,line);
		glm::vec3 projP = p - proj;
		
		return glm::dot(projP, projP);
	}

	inline float DistanceToTriangleSq(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& p)
	{
		glm::vec3 normal = glm::normalize(glm::cross((b - a), (c - a)));
		glm::vec3 pa = p - a;
		float dotp = glm::dot(pa, normal);
		//Point projected onto normal
		glm::vec3 projN = dotp * normal;
		glm::vec3 diff = p - projN;
		//Point projected onto triangle, relative to A
		glm::vec3 projA = pa - projN;



		return glm::dot(dotp, dotp);
	}

}

#endif