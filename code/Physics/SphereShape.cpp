#include "SphereShape.h"

SphereShape::SphereShape(float radius)
{
	m_type = Sphere;
	m_radius = radius;
}

glm::mat3 SphereShape::GetTensor(float mass)
{
	glm::mat3 result((2.f / 5.f) * mass * m_radius * m_radius);

	return result;
}


PhysicsDefs::AABB SphereShape::GetAABB()
{
	//glm::vec3 origin(transform[3]);
	PhysicsDefs::AABB result;

	result.max = glm::vec3(m_radius, m_radius, m_radius);
	result.min = -result.max;

	return result;
}

glm::vec3 SphereShape::GetSupportPoint(const glm::vec3& dir) const
{
	return dir*m_radius;
}