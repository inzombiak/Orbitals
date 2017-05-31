#ifndef MANIFOLD_H
#define MANIFOLD_H

#include "PhysicsDefs.h"

class Manifold
{
public:
	Manifold(IRigidBody* a, IRigidBody* b);
	
	void Solve();
	void Update(PhysicsDefs::ContactInfo* newContacts, int newContactCount);

	static const int MIN_POINTS = 4;
	PhysicsDefs::ContactInfo m_contacts[6];
	int m_contactCount = 0;

	IRigidBody* m_bodyA = 0;
	IRigidBody* m_bodyB = 0;
};

//based on Box2D Lite's ArbiterKey
struct ManifoldKey
{
	ManifoldKey(IRigidBody* a, IRigidBody* b)
	{
		if (a < b)
		{
			bodyA = a;
			bodyB = b;
		}
		else
		{
			bodyA = b;
			bodyB = a;
		}

	}

	IRigidBody* bodyA = 0;
	IRigidBody* bodyB = 0;
};

inline bool operator <(const ManifoldKey& m1, const ManifoldKey& m2)
{
	if (m1.bodyA < m2.bodyA)
		return true;

	if (m1.bodyA == m2.bodyA && m1.bodyB < m2.bodyB)
		return true;

	return false;
}

#endif