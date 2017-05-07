#include "BoxShape.h"

BoxShape::BoxShape(const glm::vec3& extents)
{
	m_extents = extents;
	m_type = Box;
}

glm::mat3 BoxShape::GetTensor(float mass)
{
	//TODO: FIX FOR SCALING
	glm::mat3 result(0.f);

	result[0][0] = mass / 12 * (m_extents.y * m_extents.y + m_extents.z * m_extents.z);
	result[1][1] = mass / 12 * (m_extents.x * m_extents.x + m_extents.z * m_extents.z);
	result[2][2] = mass / 12 * (m_extents.y * m_extents.y + m_extents.x * m_extents.x);

	return result;
}

PhysicsDefs::OBB BoxShape::GetLocalOBB()
{
	//glm::vec3 origin(transform[3]);
	PhysicsDefs::OBB result;

	result.halfExtents = m_extents * 0.5f;
	result.pos = glm::vec3(0.f);
	result.localX = glm::vec3(1.f, 0.f, 0.f);
	result.localY = glm::vec3(0.f, 1.f, 0.f);

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