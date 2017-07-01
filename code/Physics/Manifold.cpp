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
	//Based on Allen Chou's Game Physics series
	if (m_contacts.size() == 0)
	{
		for (int i = 0; i < newContactCount; ++i)
		{
			m_contacts.push_back(newContacts[i]);
		}

		return;
	}
	//Merge points, except those too close to an old point
	glm::vec3 rA, rB;
	for (int i = 0; i < newContactCount; ++i)
	{
		for (int j = 0; j < m_contacts.size(); ++j)
		{
			rA = newContacts[i].worldPointA - m_contacts[j].worldPointA;
			rB = newContacts[i].worldPointB - m_contacts[j].worldPointB;

			if (glm::dot(rA, rA) > 0.001 && glm::dot(rB, rB) > 0.001)
			{
				k++;
				m_contacts.push_back(newContacts[i]);
			}
				
		}
	
	}

	//Find deepest point
	//TODO: change to pointer
	PhysicsDefs::ContactInfo deepest;
	float currMax = -FLT_MAX;
	for (int i = 0; i < m_contacts.size(); ++i)
	{
		if (m_contacts[i].depth > currMax)
		{
			currMax = m_contacts[i].depth;
			deepest = m_contacts[i];
		}
	}

	//Find second point, furthest from the deepest point
	PhysicsDefs::ContactInfo point1;
	glm::vec3 diff;
	float distance;
	currMax = -FLT_MAX;
	for (int i = 0; i < m_contacts.size(); ++i)
	{
		diff = m_contacts[i].worldPointA - deepest.worldPointA;
		distance = glm::dot(diff, diff);
		if (distance > currMax)
		{
			currMax = distance;
			point1 = m_contacts[i];
		}
	}

	//Find third point, furthest from the line between deepest and point1
	PhysicsDefs::ContactInfo point2;
	currMax = -FLT_MAX;
	for (int i = 0; i < m_contacts.size(); ++i)
	{
		distance = DistanceToLineSq(deepest.worldPointA, point1.worldPointA, m_contacts[i].worldPointA);
		if (distance > currMax)
		{
			currMax = distance;
			point2 = m_contacts[i];
		}
	}

	//Find fourth point, furthest from triangle formed by deepest, point1 and point2
	PhysicsDefs::ContactInfo point3;
	currMax = -FLT_MAX;
	for (int i = 0; i < m_contacts.size(); ++i)
	{
		distance = DistanceToTriangleSq(deepest.worldPointA, point1.worldPointA, point2.worldPointA, m_contacts[i].worldPointA);
		if (distance > currMax)
		{
			currMax = distance;
			point3 = m_contacts[i];
		}
	}
	
	m_contacts.clear();
	m_contacts.push_back(deepest);
	m_contacts.push_back(point1);
	m_contacts.push_back(point2);
	m_contacts.push_back(point3);
} 