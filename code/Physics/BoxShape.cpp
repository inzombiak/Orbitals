#include "BoxShape.h"

BoxShape::BoxShape(const glm::vec3& extents)
{
	m_extents = extents;
	m_type = Box;
}

PhysicsDefs::AABB BoxShape::GetAABB()
{
	//glm::vec3 origin(transform[3]);
	PhysicsDefs::AABB result;
	result.max = m_extents * 0.5f;
	result.min = - m_extents * 0.5f;

	return result;
}


glm::vec3 BoxShape::GetSupportPoint(const glm::vec3& dir) const
{
	//TODO: IMPROVE
	glm::vec3 result;
	float dot, max = FLT_MIN;

	glm::vec3 extents = m_extents * 0.5f;
	std::vector<glm::vec3> vertices;
	vertices.push_back(glm::vec3(extents.x, -extents.y, -extents.z));
	vertices.push_back(glm::vec3(extents.x, extents.y, -extents.z));
	vertices.push_back(glm::vec3(extents.x, -extents.y, extents.z));
	vertices.push_back(glm::vec3(extents.x, extents.y, extents.z));
	vertices.push_back(glm::vec3(-extents.x, -extents.y, -extents.z));
	vertices.push_back(glm::vec3(-extents.x, extents.y, -extents.z));
	vertices.push_back(glm::vec3(-extents.x, -extents.y, extents.z));
	vertices.push_back(glm::vec3(-extents.x, extents.y, extents.z));

	for (int i = 0; i < vertices.size(); ++i)
	{
		dot = glm::dot(dir, vertices[i]);
		if (dot > max)
		{
			max = dot;
			result = vertices[i];
		}
	}

	return result;
}