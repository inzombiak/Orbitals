#ifndef NARROWPHASE_SAT_H
#define NARROWPHASE_SAT_H

#include "INarrowphase.h"

class NarrowphaseSAT : public INarrowphase
{
public:
	std::vector<PhysicsDefs::CollPairContactInfo> PerformCollisionResolution(const std::vector<PhysicsDefs::CollisionPair>& collisionPairs, ErrorCallBack ecb = 0);

private:

	bool SATDetectionAABB(const PhysicsDefs::AABB& aabb1, const PhysicsDefs::AABB& aabb2, PhysicsDefs::ContactInfo& contactInfo);
};

#endif