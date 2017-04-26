#include "IBroadphase.h"

class BroadphaseAABB : public IBroadphase
{
public:

	virtual void AddAABB(PhysicsDefs::AABB *aabb)
	{
		m_aabbs.push_back(aabb);
	}

	virtual void Update()
	{
		//TODO: Nothing
	}

	virtual const std::vector<PhysicsDefs::CollisionPair>& GetCollisionPairs();

protected:
	typedef std::vector<PhysicsDefs::AABB*> AABBVector;
	AABBVector m_aabbs;

};