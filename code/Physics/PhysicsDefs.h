#pragma once

#include <array>
#include <vector>

#include "../Utilities/GameDefs.h"

class ICelestialObject;
class ICollisionShape;
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

		ICollisionShape* collisionShape;
	};

	struct Contact
	{
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
	};

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

