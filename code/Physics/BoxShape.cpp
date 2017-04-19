#include "BoxShape.h"

BoxShape::BoxShape(const glm::vec3& extents)
{
	m_extents = extents;
	m_type = Box;
}

void BoxShape::GetAABB(glm::vec3& aabbMin, glm::vec3& aabbMax)
{
	//glm::vec3 origin(transform[3]);
	aabbMax = m_extents * 0.5f;
	aabbMin = - m_extents * 0.5f;
}
