#pragma once

#include <array>
#include <vector>

#include "../Utilities/GameDefs.h"

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
	
	struct RigidBodyConstructionInfo
	{
		float mass;
		float linearDamping;
		float angularDamping;
		float friction;
		float rollingFriction;
		float resititution;

		glm::mat4 transform;
		glm::vec3 localInertia;

		ICollisionShape* collisionShape = 0;
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
		float prevTangImp = 0.f;
	};

	struct SupportPoint
	{
		glm::vec3 position;
		glm::vec3 originA;
		glm::vec3 originB;
		glm::vec3 dir;
	};

	struct AABB
	{
		glm::vec3 min;
		glm::vec3 max;

		glm::mat4 worldTransfrom;
		IRigidBody* body = 0;

		bool Intersects(const AABB& other) const
		{
			if (min.x <= other.max.x && max.x >= other.min.x &&
				min.y <= other.max.y && max.y >= other.min.y &&
				min.z <= other.max.z && max.z >= other.min.z)
			{
				return true;
			}

			return false;
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

		int id;
		ICelestialObject* owner;
		virtual ICreationData* clone() const = 0;
	protected:
		virtual void SetBodyAndShape() {};
		CollisionShapeType shapeType;
		PhysicsBodyType bodyType;

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
		double sphereRadius;

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
}

