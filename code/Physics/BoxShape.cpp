#include "BoxShape.h"

BoxShape::BoxShape(const glm::vec3& extents)
{
	m_extents = extents;
	m_type = Box;
}

void BoxShape::GetAABB(const glm::mat4& transform, glm::vec3& aabbMin, glm::vec3& aabbMax)
{
	glm::vec3 origin(transform[3]);
	aabbMax = origin + m_extents;
	aabbMin = origin - m_extents
}
