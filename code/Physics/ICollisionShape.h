#ifndef I_COLLISION_SHAPE_H
#define I_COLLISION_SHAPE_H

#include "PhysicsDefs.h"
#include <glm\glm.hpp>

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

	virtual glm::mat3 GetTensor(float mass) = 0;
	virtual PhysicsDefs::OBB GetLocalOBB() = 0;
	CollisionShapeType GetType();
	virtual glm::vec3 GetSupportPoint(const glm::vec3& dir) const = 0;

protected:
	CollisionShapeType m_type;

};

#endif