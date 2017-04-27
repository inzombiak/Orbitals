#ifndef I_NARROWPHASE_H
#define I_NARROWPHASE_H

#include "IRigidBody.h"



class INarrowphase
{
public:
	virtual ~INarrowphase() {}
	typedef void(*ErrorCallBack)(std::vector<glm::vec3>);
	virtual void PerformCollisionResolution(const std::vector<PhysicsDefs::CollisionPair>& collisionPairs, ErrorCallBack ecb = 0) = 0;
};

#endif