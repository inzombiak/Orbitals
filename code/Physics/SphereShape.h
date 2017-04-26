#ifndef SPHERE_SHAPE_H
#define SPHERE_SHAPE_H

#include "ICollisionShape.h"

class SphereShape : public ICollisionShape
{

public:
	SphereShape(float radius);
	PhysicsDefs::AABB GetAABB() override;
	glm::vec3 GetSupportPoint(const glm::vec3& dir) const;

private:
	
	float m_radius;
};

#endif