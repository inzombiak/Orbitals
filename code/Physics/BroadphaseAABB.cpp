#include "BroadphaseAABB.h"

using PhysicsDefs::AABB;

const std::vector<PhysicsDefs::CollisionPair>& BroadphaseAABB::GetCollisionPairs()
{
	m_colliderPairs.clear();
	AABB otherAABBtoLocal;
	glm::mat4 aabb1Inverse;
	for (int i = 0; i < m_aabbs.size(); ++i)
	{
		aabb1Inverse = glm::inverse(m_aabbs[i]->worldTransfrom);
		for (int j = i + 1; j < m_aabbs.size(); ++j)
		{
			otherAABBtoLocal.min = glm::vec3(aabb1Inverse * m_aabbs[j]->worldTransfrom * glm::vec4(m_aabbs[j]->min, 1.f));
			otherAABBtoLocal.max = glm::vec3(aabb1Inverse * m_aabbs[j]->worldTransfrom * glm::vec4(m_aabbs[j]->max, 1.f));

			if (m_aabbs[i]->Intersects(otherAABBtoLocal))
			{
				m_colliderPairs.emplace_back(std::make_pair(m_aabbs[i]->body, m_aabbs[j]->body));
			}
		}
	}

	return m_colliderPairs;
}
