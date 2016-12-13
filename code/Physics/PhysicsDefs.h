#pragma once

#include <array>
#include <vector>

#include "../Utilities/GameDefs.h"

class Object;
namespace PhysicsDefs
{
	enum PhysicsEntityTypes
	{
		Box,
		Sphere,
		Compound,
		Constraint,
	};

	enum PhysicsBodyType
	{
		RigidBody,
		CompoundBody,
		NULLBody,
	};

	class ICreationData
	{
	public:
		virtual ~ICreationData() {};

		PhysicsEntityTypes GetShape()
		{
			return shapeType;
		}

		PhysicsBodyType GetBody()
		{
			return bodyType;
		}

		int id;
		Object* owner;
		virtual ICreationData* clone() const = 0;
	protected:
		virtual void SetBodyAndShape() {};
		PhysicsEntityTypes shapeType;
		PhysicsBodyType bodyType;

	};

	class IRigidBodyCreationData : public ICreationData
	{
	public:
		virtual ~IRigidBodyCreationData() {};

		double mass;
		double friction;
		double rollingFriction;
		std::array<double, 3> rotationAngles;
		std::array<double, 3> msPosition;
	};

	class BoxCreationData : public IRigidBodyCreationData
	{
	public:
		BoxCreationData()
		{
			SetBodyAndShape();
		}

		virtual ~BoxCreationData() {};
		std::array<double, 3> boxDimensions;
	
		BoxCreationData* clone() const
		{
			return new BoxCreationData(*this);
		}

	protected:
		void SetBodyAndShape() override
		{
			ICreationData::shapeType = PhysicsEntityTypes::Box;
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
			ICreationData::shapeType = PhysicsEntityTypes::Sphere;
			ICreationData::bodyType = PhysicsBodyType::RigidBody;
		}
	};

}

