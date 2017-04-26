#ifndef I_COLLISION_SHAPE_H
#define I_COLLISION_SHAPE_H

#include <glm\glm.hpp>
#include "PhysicsDefs.h"

enum CollisionShapeType
{
	Sphere = 1,
	Box = 2,
};

class ICollisionShape
{
public:

	virtual ~ICollisionShape()
	{
		;
	}

	virtual PhysicsDefs::AABB GetAABB() = 0;
	CollisionShapeType GetType();
	virtual glm::vec3 GetSupportPoint(const glm::vec3& dir) const = 0;

protected:
	CollisionShapeType m_type;

};

#endif