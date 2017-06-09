#ifndef NARROWPHASE_SAT_H
#define NARROWPHASE_SAT_H

#include "INarrowphase.h"

class NarrowphaseSAT : public INarrowphase
{
public:
	std::vector<Manifold> CheckCollision(const std::vector<PhysicsDefs::CollisionPair>& collisionPairs, ErrorCallBack ecb = 0) override;

private:

	bool SATDetectionAABB(IRigidBody* body1, IRigidBody* body2, PhysicsDefs::ContactInfo& contactInfo);
	bool SATDetectionOBB(IRigidBody* body1, IRigidBody* body2, Manifold& manifold);

	//Used for culling manifold points
	void CullPoints(std::vector<glm::vec2> points, int finalCount, int firstEntry, int culledPoints[]);
};

#endif