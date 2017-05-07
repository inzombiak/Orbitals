#ifndef BOX_SHAPE_H
#define BOX_SHAPE_H

#include "ICollisionShape.h"

class BoxShape : public ICollisionShape
{
public:
	BoxShape(const glm::vec3& extents);
	glm::mat3 GetTensor(float mass) override;
	PhysicsDefs::OBB GetLocalOBB() override;
	glm::vec3 GetSupportPoint(const glm::vec3& dir) const;

private:
	glm::vec3 m_extents;
};

#endif