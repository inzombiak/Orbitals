#ifndef NARROWPHASE_SAT_H
#define NARROWPHASE_SAT_H

#include "INarrowphase.h"

class NarrowphaseSAT : public INarrowphase
{
public:
	void PerformCollisionResolution(const std::vector<PhysicsDefs::CollisionPair>& collisionPairs, ErrorCallBack ecb = 0);

private:

	bool SATDetectionAABB(const PhysicsDefs::AABB& aabb1, const PhysicsDefs::AABB& aabb2, glm::vec3& normalOut, float& depthOut);
};

#endif