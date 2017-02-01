#include "SphereShape.h"

SphereShape::SphereShape(float radius)
{
	m_type = Sphere;
	m_radius = radius;
}

void SphereShape::GetAABB(const glm::mat4& transform, glm::vec3& aabbMin, glm::vec3& aabbMax)
{
	glm::vec3 origin(transform[3]);
	aabbMax = origin + m_radius;
	aabbMin = origin - m_radius;


}