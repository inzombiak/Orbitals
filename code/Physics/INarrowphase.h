#ifndef I_NARROWPHASE_H
#define I_NARROWPHASE_H

#include "IRigidBody.h"

class INarrowphase
{
public:
	virtual ~INarrowphase() {}

	virtual void PerformCollisionResolution(const std::vector<PhysicsDefs::CollisionPair>& collisionPairs) = 0;
};

#endif