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

	virtual void GetAABB(const glm::mat4& transform, glm::vec3& aabbMin, glm::vec3& aabbMax) = 0;
	CollisionShapeType GetType();

protected:
	CollisionShapeType m_type;

};

#endif