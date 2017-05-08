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

PhysicsDefs::OBB SphereShape::GetLocalOBB()
{
	//glm::vec3 origin(transform[3]);
	PhysicsDefs::OBB result;

	result.halfExtents = glm::vec3(m_radius, m_radius, m_radius);
	result.pos = glm::vec3(0.f);

	result.localAxes[0] = glm::vec3(1.f, 0.f, 0.f);
	result.localAxes[1] = glm::vec3(0.f, 1.f, 0.f);
	result.localAxes[2] = glm::vec3(0.f, 0.f, 1.f);

	return result;
}

glm::vec3 SphereShape::GetSupportPoint(const glm::vec3& dir) const
{
	return dir*m_radius;
}