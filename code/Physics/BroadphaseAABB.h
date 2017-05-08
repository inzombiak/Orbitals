#include "IBroadphase.h"

class BroadphaseAABB : public IBroadphase
{
public:

	virtual void AddAABB(const PhysicsDefs::AABB *aabb)
	{
		m_aabbs.push_back(aabb);
	}

	virtual void Update()
	{
		//TODO: Nothing
	}

	virtual const std::vector<PhysicsDefs::CollisionPair>& GetCollisionPairs();

protected:
	typedef std::vector<const PhysicsDefs::AABB*> AABBVector;
	AABBVector m_aabbs;

};