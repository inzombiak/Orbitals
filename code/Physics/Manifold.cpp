#include "Manifold.h"

using namespace PhysicsDefs;

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

void Manifold::Solve()
{
	
}

void Manifold::Update(PhysicsDefs::ContactInfo* newContacts, int newContactCount)
{
	ContactInfo mergedContacts[4];

	ContactInfo *newC, *oldC , *c;
	int k = -1;
	//Merge the old and new contacts
	for (int i = 0; i < newContactCount; ++i)
	{
		newC = newContacts + i;
		k = -1;
		for (int j = 0; j < m_contactCount; ++j)
		{
			oldC = m_contacts + j;
			if ()
			{
				k = j;
				break;
			}
		}

		//If an old and new contact are the same, copy over data
		if (k > -1)
		{
			c = mergedContacts + i;
			oldC = m_contacts + k;

			c->prevNormalImp = oldC->prevNormalImp;
			c->prevTangImp = oldC->prevTangImp;

		}
		else
		{
			mergedContacts[i] = newContacts[i];
		}

	}

	//Assign the new contacts
	for (int i = 0; i < newContactCount; ++i)
	{
		m_contacts[i] = mergedContacts[i];
	}

	//Set count
	m_contactCount = newContactCount;
}