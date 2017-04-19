#ifndef BOX_SHAPE_H
#define BOX_SHAPE_H

#include "ICollisionShape.h"

class BoxShape : public ICollisionShape
{
public:
	BoxShape(const glm::vec3& extents);
	void GetAABB(glm::vec3& aabbMin, glm::vec3& aabbMax) override;

private:
	glm::vec3 m_extents;
};

#endif