#ifndef I_BROADPHASE_H
#define I_BROADPHASE_H

#include "IRigidBody.h"

///TODO: ADD AABB REMOVAL
class IBroadphase
{
public:
	virtual ~IBroadphase() {}

	virtual void AddAABB(const PhysicsDefs::AABB *aabb) = 0;
	virtual void Update() = 0;

	virtual const std::vector<PhysicsDefs::CollisionPair>& GetCollisionPairs() = 0;

protected:
	
	std::vector<PhysicsDefs::CollisionPair> m_colliderPairs;
};

#endif