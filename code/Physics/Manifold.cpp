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
	std::vector<ContactInfo> mergedContacts;

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
			rA = newContacts[i].localPointA - m_contacts[j].localPointA;
			rB = newContacts[i].localPointB - m_contacts[j].localPointB;

			if (glm::dot(rA, rA) > 0.001 && glm::dot(rB, rB) > 0.001)
			{
				m_contacts.push_back(newContacts[i]);
			}
				
		}
	
	}

	//Find deepest point
	//TODO: change to pointer
	//TODO: get rid of the bool
	bool found = false;
	PhysicsDefs::ContactInfo deepest;
	float currMax = -FLT_MAX;
	for (int i = 0; i < m_contacts.size(); ++i)
	{
		if (m_contacts[i].depth > currMax)
		{
			currMax = m_contacts[i].depth;
			deepest = m_contacts[i];
			found = true;
		}
	}

	if (found)
		mergedContacts.push_back(deepest);

	//Find second point, furthest from the deepest point
	PhysicsDefs::ContactInfo point1;
	glm::vec3 diff;
	float distance;
	currMax = -FLT_MAX;
	found = false;
	for (int i = 0; i < m_contacts.size(); ++i)
	{
		diff = m_contacts[i].localPointA - deepest.localPointA;
		distance = glm::dot(diff, diff);
		if (distance > currMax)
		{
			currMax = distance;
			point1 = m_contacts[i];
			found = true;
		}
	}

	if (found)
		mergedContacts.push_back(point1);

	//Find third point, furthest from the line between deepest and point1
	PhysicsDefs::ContactInfo point2;
	currMax = -FLT_MAX;
	found = false;
	for (int i = 0; i < m_contacts.size(); ++i)
	{
		distance = DistanceToLineSq(deepest.localPointA, point1.localPointA, m_contacts[i].localPointA);
		if (distance > currMax)
		{
			currMax = distance;
			point2 = m_contacts[i];
			found = true;
		}
	}

	if (found)
		mergedContacts.push_back(point2);

	//Find fourth point, furthest from triangle formed by deepest, point1 and point2
	PhysicsDefs::ContactInfo point3;
	currMax = -FLT_MAX;
	found = false;
	for (int i = 0; i < m_contacts.size(); ++i)
	{
		distance = DistanceToTriangleSq(deepest.localPointA, point1.localPointA, point2.localPointA, m_contacts[i].localPointA);
		if (distance > currMax)
		{
			currMax = distance;
			point3 = m_contacts[i];
			found = true;
		}
	}

	if (found)
		mergedContacts.push_back(point3);
	
	m_contacts = mergedContacts;
} 