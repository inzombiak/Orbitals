#ifndef NARROWPHASE_SAT_H
#define NARROWPHASE_SAT_H

#include "INarrowphase.h"

class NarrowphaseSAT : public INarrowphase
{
public:
	std::vector<PhysicsDefs::CollPairContactInfo> CheckCollision(const std::vector<PhysicsDefs::CollisionPair>& collisionPairs, ErrorCallBack ecb = 0);

private:

	bool SATDetectionAABB(IRigidBody* body1, IRigidBody* body2, PhysicsDefs::ContactInfo& contactInfo);
	bool SATDetectionOBB(IRigidBody* body1, IRigidBody* body2, PhysicsDefs::ContactInfo& contactInfo);
};

#endif