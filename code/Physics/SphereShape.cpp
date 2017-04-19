#include "SphereShape.h"

SphereShape::SphereShape(float radius)
{
	m_type = Sphere;
	m_radius = radius;
}

void SphereShape::GetAABB(glm::vec3& aabbMin, glm::vec3& aabbMax)
{
	//glm::vec3 origin(transform[3]);
	aabbMax = glm::vec3(m_radius, m_radius, m_radius);
	aabbMin = -aabbMax;
}