#ifndef SPHERE_SHAPE_H
#define SPHERE_SHAPE_H

#include "ICollisionShape.h"

class SphereShape : public ICollisionShape
{

public:
	SphereShape(float radius);
	void GetAABB(const glm::mat4& transform, glm::vec3& aabbMin, glm::vec3& aabbMax) override;

private:
	
	float m_radius;
};

#endif