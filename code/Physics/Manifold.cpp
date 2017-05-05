#include "Manifold.h"

Manifold::Manifold(IRigidBody* a, IRigidBody* b)
{
	if (a < b)
	{
		m_bodyA = a;
		m_bodyB = b;
	}
	else
	{
		m_bodyA = b;
		m_bodyB = a;
	}

}

