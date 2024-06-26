#include "NarrowphaseSAT.h"
#include "glm\gtc\matrix_transform.hpp"
#include "glm\gtx\quaternion.hpp"

std::vector<Manifold> NarrowphaseSAT::CheckCollision(const std::vector<PhysicsDefs::CollisionPair>& collisionPairs, ErrorCallBack ecb)
{
	std::vector<Manifold> result;
	for (int i = 0; i < collisionPairs.size(); ++i)
	{

		Manifold manifold(collisionPairs[i].first, collisionPairs[i].second);
		if (SATDetectionOBB(manifold.m_bodyA, manifold.m_bodyB, manifold))
		{
			//Since we moved everything into A's space we neeed to move it out 
			result.push_back(manifold);
		}
	}

	return result;
}

bool NarrowphaseSAT::SATDetectionAABB(IRigidBody* body1, IRigidBody* body2, PhysicsDefs::ContactInfo& contactInfo)
{

	contactInfo.depth = FLT_MAX;
	float currentAxisDepth = 0;
	bool test = true;
	int sign;
	glm::vec3 contactPointA, contactNormal;

	PhysicsDefs::AABB aabb1, aabb2;

	aabb1 = body1->GetAABB();
	aabb2 = body2->GetAABB();

	glm::mat4 interpolationTrans1 = aabb1.worldTransform;
	glm::mat4 interpolationTrans2 = aabb2.worldTransform;
	glm::mat4 bodyBtoA = glm::inverse(interpolationTrans1) * interpolationTrans2;

	aabb2.min = glm::vec3(bodyBtoA * glm::vec4(aabb2.min, 1.f));
	aabb2.max = glm::vec3(bodyBtoA * glm::vec4(aabb2.max, 1.f));

	if (aabb1.min.x <= aabb2.max.x && aabb1.max.x >= aabb2.min.x)
	{
		currentAxisDepth = std::min(aabb1.max.x, aabb2.max.x) - std::max(aabb1.min.x, aabb2.min.x);
		contactNormal = glm::vec3(1.f, 0, 0);
		sign = (aabb2.min.x - aabb1.min.x < 0) ? -1 : 1;
		
		contactPointA.x = sign * (currentAxisDepth);
		contactNormal *= sign;

		if (currentAxisDepth < contactInfo.depth)
		{
			contactInfo.depth = currentAxisDepth;
			contactInfo.normal = contactNormal;
		}
	}
	else
		return false;

	if (aabb1.min.y <= aabb2.max.y && aabb1.max.y >= aabb2.min.y)
	{
		currentAxisDepth = std::min(aabb1.max.y, aabb2.max.y) - std::max(aabb1.min.y, aabb2.min.y);
		contactNormal = glm::vec3(0, 1.0f, 0);
		sign = (aabb2.min.y - aabb1.min.y < 0) ? -1 : 1;
		
		contactPointA.y = sign * (aabb1.max.y - currentAxisDepth);
		contactNormal *= sign;

		if (currentAxisDepth < contactInfo.depth)
		{
			contactInfo.depth = currentAxisDepth;
			contactInfo.normal = contactNormal;
		}
	}
	else
		return false;

	if (aabb1.min.z <= aabb2.max.x && aabb1.max.z >= aabb2.min.z)
	{
		currentAxisDepth = std::min(aabb1.max.z, aabb2.max.z) - std::max(aabb1.min.z, aabb2.min.z);
		contactNormal = glm::vec3(0, 0, 1.f);
		sign = (aabb2.min.z - aabb1.min.z < 0) ? -1 : 1;

		contactPointA.z = sign * (currentAxisDepth);
		contactNormal *= sign;

		if (currentAxisDepth < contactInfo.depth)
		{
			contactInfo.depth = currentAxisDepth;
			contactInfo.normal = contactNormal;
		}
	}
	else
		return false;

	if (contactInfo.depth < FLT_EPSILON)
		return false;

	contactInfo.localPointA = contactPointA;

	//contactInfo.worldPointB = glm::vec3(interpolationTrans1 *  glm::vec4(contactInfo.localPointA - contactInfo.normal * contactInfo.depth, 1.f));

	//contactInfo.localPointA = glm::vec3(glm::inverse(interpolationTrans2) *  glm::vec4(contactInfo.worldPointB, 1.f));
	contactInfo.worldPointA = glm::vec3(interpolationTrans1 * glm::vec4(contactInfo.localPointA, 1));

	return true;
}

//Based on Real-time Collision Detection by Christer Ericson, pg. 103
bool NarrowphaseSAT::SATDetectionOBB(IRigidBody* body1, IRigidBody* body2, Manifold& manifold)
{
	PhysicsDefs::ContactInfo contactInfo;
	float ra, rb, depth;
	const float fudge_factor = 1.05f;
	contactInfo.depth = -FLT_MAX;
	glm::mat3 R, absR;
	PhysicsDefs::OBB obb1 = body1->GetOBB();
	PhysicsDefs::OBB obb2 = body2->GetOBB();

	/*
	Ranges from 1-15
	1-3: Face of A
	4-6: Face of B
	6-15: Some combinaiton of edges

	0: No collision
	*/
 	int featureID = 0;
	bool invertNormal = false;
	//TODO: WHY?
	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			R[i][j] = glm::dot(obb1.localAxes[i], obb2.localAxes[j]);
			absR[i][j] = std::abs(R[i][j]) + FLT_EPSILON;
		}
	}
	//R = glm::transpose(R);
	glm::vec3 trans = obb2.pos - obb1.pos;
	trans = trans * obb1.localAxes;

	float TL;
	//Text A0, A1 and A2
	for (int i = 0; i < 3; ++i)
	{
		ra = obb1.halfExtents[i];
		rb = obb2.halfExtents[0] * absR[i][0] + obb2.halfExtents[1] * absR[i][1] + obb2.halfExtents[2] * absR[i][2];
		TL = trans[i];

		depth = std::abs(TL) - (ra + rb);
		
		if (depth > 0)
			return false;

		if (depth < -0.00001f && depth * fudge_factor > contactInfo.depth)
		{
			invertNormal = TL < 0;
			contactInfo.depth = depth;
			contactInfo.normal = obb1.localAxes[i];
			featureID = i + 1;
		}
	}

	//Test B0, B1, B2
	for (int i = 0; i < 3; ++i)
	{
		ra = obb1.halfExtents[0] * absR[0][i] + obb1.halfExtents[1] * absR[1][i] + obb1.halfExtents[2] * absR[2][i];
		rb = obb2.halfExtents[i];
		TL = trans[0] * R[0][i] + trans[1] * R[1][i] + trans[2] * R[2][i];
		depth = std::abs(TL) - (ra + rb);

		if (depth > 0)
			return false;

		if (depth < -0.00001f && depth * fudge_factor > contactInfo.depth)
		{
			invertNormal = TL < 0;
			contactInfo.depth = depth;
			contactInfo.normal = obb2.localAxes[i];
			featureID = i + 4;
		}
	}

	glm::vec3 normal;
	//Test A0 x B0
	ra = obb1.halfExtents[1] * absR[2][0] + obb1.halfExtents[2] * absR[1][0];
	rb = obb2.halfExtents[1] * absR[0][2] + obb2.halfExtents[2] * absR[0][1];
	TL = trans[2] * R[1][0] - trans[1] * R[2][0];
	depth = std::abs(TL) - (ra + rb);
	if (depth > 0)
		return false;
	else
	{
		normal = glm::cross(obb1.localAxes[0], obb2.localAxes[0]);
		if (depth > 0.00001f && depth * fudge_factor > contactInfo.depth && normal != glm::vec3(0.f))
		{
			invertNormal = TL < 0;
			contactInfo.depth = depth;
			contactInfo.normal = normal;
			featureID = 7;
		}
	}

	//Test A0 x B1
	ra = obb1.halfExtents[1] * absR[2][1] + obb1.halfExtents[2] * absR[1][1];
	rb = obb2.halfExtents[0] * absR[0][2] + obb2.halfExtents[2] * absR[0][0];
	TL = trans[2] * R[1][1] - trans[1] * R[2][1];

	depth = std::abs(TL) - (ra + rb);
	if (depth > 0)
		return false;
	else
	{

		normal = glm::cross(obb1.localAxes[0], obb2.localAxes[1]);
		if (depth > 0.00001f && depth * fudge_factor > contactInfo.depth && normal != glm::vec3(0.f))
		{
			invertNormal = TL < 0;
			contactInfo.depth = depth;
			contactInfo.normal = normal;
			featureID = 8;
		}
	}

	//Test A0 x B2
	ra = obb1.halfExtents[1] * absR[2][2] + obb1.halfExtents[2] * absR[1][2];
	rb = obb2.halfExtents[0] * absR[0][1] + obb2.halfExtents[1] * absR[0][0];
	TL = trans[2] * R[1][2] - trans[1] * R[2][2];

	depth = std::abs(TL) - (ra + rb);
	if (depth > 0)
		return false;
	else
	{
		normal = glm::cross(obb1.localAxes[0], obb2.localAxes[2]);
		if (depth > 0.00001f && depth * fudge_factor > contactInfo.depth && normal != glm::vec3(0.f))
		{
			invertNormal = TL < 0;
			contactInfo.depth = depth;
			contactInfo.normal = normal;
			featureID = 9;
		}
	}

	//Test A1 x B0
	ra = obb1.halfExtents[0] * absR[2][0] + obb1.halfExtents[2] * absR[0][0];
	rb = obb2.halfExtents[1] * absR[1][2] + obb2.halfExtents[2] * absR[1][1];
	TL = trans[0] * R[2][0] - trans[2] * R[0][0];

	depth = std::abs(TL) - (ra + rb);
	if (depth > 0)
		return false;
	else
	{
		normal = glm::cross(obb1.localAxes[1], obb2.localAxes[0]);
		if (depth > 0.00001f && depth * fudge_factor > contactInfo.depth && normal != glm::vec3(0.f))
		{
			invertNormal = TL < 0;
			contactInfo.depth = depth;
			contactInfo.normal = normal;
			featureID = 10;
		}
	}

	//Test A1 x B1
	ra = obb1.halfExtents[0] * absR[2][1] + obb1.halfExtents[2] * absR[0][1];
	rb = obb2.halfExtents[0] * absR[1][2] + obb2.halfExtents[2] * absR[1][0];
	TL = trans[0] * R[2][1] - trans[2] * R[0][1];

	depth = std::abs(TL) - (ra + rb);
	if (depth > 0)
		return false;
	else
	{
		normal = glm::cross(obb1.localAxes[1], obb2.localAxes[1]);
		if (depth > 0.00001f && depth * fudge_factor > contactInfo.depth && normal != glm::vec3(0.f))
		{
			invertNormal = TL < 0;
			contactInfo.depth = depth;
			contactInfo.normal = normal;
			featureID = 11;
		}
	}

	//Test A1 x B2
	ra = obb1.halfExtents[0] * absR[2][2] + obb1.halfExtents[2] * absR[0][2];
	rb = obb2.halfExtents[0] * absR[1][1] + obb2.halfExtents[2] * absR[1][0];
	TL = trans[0] * R[2][2] - trans[2] * R[0][2];

	depth = std::abs(TL) - (ra + rb);
	if (depth > 0)
		return false;
	else
	{
		normal = glm::cross(obb1.localAxes[1], obb2.localAxes[2]);
		if (depth > 0.00001f && depth * fudge_factor > contactInfo.depth && normal != glm::vec3(0.f))
		{
			invertNormal = TL < 0;
			contactInfo.depth = depth;
			contactInfo.normal = normal;
			featureID = 12;
		}
	}

	//Test A2 x B0
	ra = obb1.halfExtents[0] * absR[1][0] + obb1.halfExtents[1] * absR[0][0];
	rb = obb2.halfExtents[1] * absR[2][2] + obb2.halfExtents[2] * absR[2][1];
	TL = trans[1] * R[0][0] - trans[0] * R[1][0];
	depth = std::abs(TL) - (ra + rb);
	if (depth > 0)
		return false;
	else
	{
		normal = glm::cross(obb1.localAxes[2], obb2.localAxes[0]);
		if (depth > 0.00001f && depth * fudge_factor > contactInfo.depth && normal != glm::vec3(0.f))
		{
			invertNormal = TL < 0;
			contactInfo.depth = depth;
			contactInfo.normal = normal;
			featureID = 13;
		}
	}

	//Test A2 x B1
	ra = obb1.halfExtents[0] * absR[1][1] + obb1.halfExtents[1] * absR[0][1];
	rb = obb2.halfExtents[0] * absR[2][2] + obb2.halfExtents[2] * absR[2][0];
	TL = trans[1] * R[0][1] - trans[0] * R[1][1];

	depth = std::abs(TL) - (ra + rb);
	if (depth > 0)
		return false;
	else
	{
		normal = glm::cross(obb1.localAxes[2], obb2.localAxes[1]);
		if (depth > 0.00001f && depth * fudge_factor > contactInfo.depth && normal != glm::vec3(0.f))
		{
			invertNormal = TL < 0;
			contactInfo.depth = depth;
			contactInfo.normal = normal;
			featureID = 14;
		}
	}

	//Test A2 x B2
	ra = obb1.halfExtents[0] * absR[1][2] + obb1.halfExtents[1] * absR[0][2];
	rb = obb2.halfExtents[0] * absR[2][1] + obb2.halfExtents[1] * absR[2][0];
	TL = trans[1] * R[0][2] - trans[0] * R[1][2];

	depth = std::abs(TL) - (ra + rb);
	if (depth > 0)
		return false;
	else
	{
		normal = glm::cross(obb1.localAxes[2], obb2.localAxes[2]);
		if (depth > 0.00001f && depth * fudge_factor > contactInfo.depth && normal != glm::vec3(0.f))
		{
			invertNormal = TL < 0;
			contactInfo.depth = depth;
			contactInfo.normal = normal;
			featureID = 15;
		}
	}

	if (invertNormal)
	{
		contactInfo.normal *= -1;
	}

	//Create manifold
	//This is based on ODE's dBoxBox function, found in the box.cpp file

	//Rotations, positions and side lengths of bodies 1 and 2
	glm::vec3 pos1, pos2, extents1, extents2;
	glm::mat3 rot1, rot2;

	glm::vec3 faceNorm = contactInfo.normal, referenceNormal, absReferenceNormal;

	//In this case the normal will be pointing from 2 to 1 so we flip it
	if (featureID > 3)
	{
		faceNorm = -contactInfo.normal;
	}

	if (featureID > 6)
	{
		glm::vec3 pa;
		float sign;

		pa = obb1.pos;
		for (int i = 0; i < 3; ++i)
		{
			sign = glm::dot(faceNorm, obb1.localAxes[i]) > 0 ? 1.f : -1.f;
			for (int j = 0; j < 3; ++j)
			{
				pa[i] += sign * obb1.halfExtents[j] * obb1.localAxes[i][j];
			}
		}

		glm::vec3 pb;
		pb = obb2.pos;
		for (int i = 0; i < 3; ++i)
		{
			sign = glm::dot(faceNorm, obb2.localAxes[i]) > 0 ? 1.f : -1.f;
			for (int j = 0; j < 3; ++j)
			{
				pb[i] += sign * obb2.halfExtents[j] * obb2.localAxes[i][j];
			}
		}

		float alpha, beta;
		glm::vec3 ua, ub;
		ua = obb1.localAxes[(featureID - 7) / 3];
		ub = obb2.localAxes[(featureID - 7) / 3];

		PhysicsDefs::RayClosestApproach(pa, ua, pb, ub, alpha, beta);
		pa += ua * alpha;
		pb += ub * beta;

		contactInfo.worldPos = 0.5f * (pa + pb);
		OTransform invTrans1 = body1->GetInterpolationTransform().Inverse(), invTrans2 = body2->GetInterpolationTransform().Inverse();
		contactInfo.localPointA = invTrans1 * contactInfo.worldPos;
		contactInfo.localPointB = invTrans2 * contactInfo.worldPos;
		manifold.Update(&contactInfo, 1);
		return true;
	}

	//Reference face is on body1
	if (featureID <= 3)
	{
		
		rot1[0] = obb1.localAxes[0];
		rot1[1] = obb1.localAxes[1];
		rot1[2] = obb1.localAxes[2];
		rot2[0] = obb2.localAxes[0];
		rot2[1] = obb2.localAxes[1];
		rot2[2] = obb2.localAxes[2];

		pos1 = obb1.pos;
		pos2 = obb2.pos;
		extents1 = obb1.halfExtents;
		extents2 = obb2.halfExtents;
	}
	else
	{
		rot1[0] = obb2.localAxes[0];
		rot1[1] = obb2.localAxes[1];
		rot1[2] = obb2.localAxes[2];
		rot2[0] = obb1.localAxes[0];
		rot2[1] = obb1.localAxes[1];
		rot2[2] = obb1.localAxes[2];
		pos1 = obb2.pos;
		pos2 = obb1.pos;
		extents1 = obb2.halfExtents;
		extents2 = obb1.halfExtents;
	}

	//Rotate the norm
	//TODO: IS THIS RIGHT?
	referenceNormal = glm::transpose(rot2) * faceNorm;

	//Calculate absolute normal
	absReferenceNormal.x = fabs(referenceNormal.x);
	absReferenceNormal.y = fabs(referenceNormal.y);
	absReferenceNormal.z = fabs(referenceNormal.z);

	//Find largest component of the absolute normal, which is the normal for the
	//incident face, other numbers of the face are stored in incFace1 and incFace2
	int incFace1, incFace2, largestAbsNorm;
	if (absReferenceNormal.y > absReferenceNormal.x)
	{
		if (absReferenceNormal.y > absReferenceNormal.z)
		{
			incFace1 = 0;
			largestAbsNorm = 1;
			incFace2 = 2;
		}
		else
		{
			incFace1 = 0;
			incFace2 = 1;
			largestAbsNorm = 2;
		}
	}
	else
	{
		if (absReferenceNormal.x > absReferenceNormal.z)
		{
			largestAbsNorm = 0;
			incFace1 = 1;
			incFace2 = 2;
		}
		else
		{
			incFace1 = 0;
			incFace2 = 1;
			largestAbsNorm = 2;
		}
	}

	//Compute center of the incident face
	glm::vec3 center = pos2 - pos1;
	glm::vec3 modifier = extents2[largestAbsNorm] * glm::vec3(rot2[largestAbsNorm]);// [], rot2[1][largestAbsNorm], rot2[2][largestAbsNorm]);
	if (referenceNormal[largestAbsNorm] < 0)
		center += modifier;
	else
		center -= modifier;

	//Find the normal and non-normal axis numbers of the reference box
	//TODO: WHAT?
	int codeN, code1, code2;
	if (featureID <= 3)
		codeN = featureID - 1;
	else
		codeN = featureID - 4;

	if (codeN == 0)
	{
		code1 = 1;
		code2 = 2;
	}
	else if (codeN == 1)
	{
		code1 = 0;
		code2 = 2;
	}
	else
	{
		code1 = 0;
		code2 = 1;
	}

	//Find the 4 corners of the incident face
	std::vector<glm::vec2> quad;
	quad.resize(4, glm::vec2(0, 0));
	float c1, c2, m11, m12, m21, m22;
	//	c1 = glm::dot(center, glm::vec3(rot1[0][code1], rot1[1][code1], rot1[2][code1]));
	//	c2 = glm::dot(center, glm::vec3(rot1[0][code2], rot1[1][code2], rot1[2][code2]));

	c1 = glm::dot(center, rot1[code1]);
	c2 = glm::dot(center, rot1[code2]);

	m11 = glm::dot(rot1[code1], rot2[incFace1]);
	m12 = glm::dot(rot1[code1], rot2[incFace2]);
	m21 = glm::dot(rot1[code2], rot2[incFace1]);
	m22 = glm::dot(rot1[code2], rot2[incFace2]);
	{
		float k1 = m11*extents2[incFace1];
		float k2 = m21*extents2[incFace1];
		float k3 = m12*extents2[incFace2];
		float k4 = m22*extents2[incFace2];
		quad[0].x = c1 - k1 - k3;
		quad[0].y = c2 - k2 - k4;
		quad[1].x = c1 - k1 + k3;
		quad[1].y = c2 - k2 + k4;
		quad[2].x = c1 + k1 + k3;
		quad[2].y = c2 + k2 + k4;
		quad[3].x = c1 + k1 - k3;
		quad[3].y = c2 + k2 - k4;
	}
	glm::vec2 diff = quad[2] - quad[0];
	//Find size of reference face
	std::vector<glm::vec2> rect;
	rect.resize(4, glm::vec2(0, 0));
	rect[0] = glm::vec2(-extents1[code1], -extents1[code2]);
	rect[1] = glm::vec2(-extents1[code1], extents1[code2]);
	rect[2] = glm::vec2(extents1[code1], extents1[code2]);
	rect[3] = glm::vec2(extents1[code1], -extents1[code2]);

	//Intersect faces
	std::vector<glm::vec2> ret;
	if (extents1[code1] * extents1[code2] < std::abs(diff.x * diff.y))
		ret = PhysicsDefs::ClipPolygon(rect, quad);
	else
		ret = PhysicsDefs::ClipPolygon(quad, rect);

	if (ret.size() < 1)
		return false;
	ret.resize(ret.size());
	glm::vec3 point[16];
	float dep[16];
	float det1 = 1.0f / (m11 * m22 - m12 * m21);

	m11 *= det1;
	m12 *= det1;
	m21 *= det1;
	m22 *= det1;

	//Number of penetrating contact points found
	int cnum = 0;
	for (int j = 0; j < ret.size(); ++j)
	{
		float k1 = m22*(ret[j].x - c1) - m12*(ret[j].y - c2);
		float k2 = -m21*(ret[j].x - c1) + m11*(ret[j].y - c2);
		point[cnum] = center + k1 * rot2[incFace1] + k2 * rot2[incFace2];
		dep[cnum] = extents1[codeN] - glm::dot(faceNorm, point[cnum]);

		if (dep[cnum] >= 0)
		{
			ret[cnum].x = ret[j].x;
			ret[cnum].y = ret[j].y;

			cnum++;
		}

	}

	//TODO: ERROR
	if (cnum < 1)
	{
		return 0;
	}

	//TODO: Make this variable?
	int maxc = 4;
	if (maxc > cnum)
		maxc = cnum;
	if (maxc < 1)
		maxc = 1;

	//We have less contacts tha we need so we use all fo them
	PhysicsDefs::ContactInfo* contacts;
	int count;
	if (cnum <= maxc)
	{
		contacts = new PhysicsDefs::ContactInfo[cnum];
		for (int i = 0; i < cnum; ++i)
		{
			contacts[i].worldPos = point[i] + pos1;
			contacts[i].depth = dep[i];
			contacts[i].normal = contactInfo.normal;
		}

		count = cnum;
	}
	else
	{
		//We have more so we cull them

		//Find deepest point
		int i1 = 0;
		float maxDepth = dep[0];
		for (int i = 1; i < cnum; ++i)
		{
			if (dep[i] > maxDepth)
			{
				maxDepth = dep[i];
				i1 = i;
			}
		}

		int iret[8];
		CullPoints(ret, maxc, i1, iret);
		contacts = new PhysicsDefs::ContactInfo[maxc];
		for (int j = 0; j < maxc; ++j)
		{
			contacts[j].worldPos = point[iret[j]] + pos1;
			contacts[j].depth = dep[iret[j]];
			contacts[j].normal = contactInfo.normal;
		}
		count = maxc;
	}

	//"Initalize" the contacts by setting the position in local-space for each body
	OTransform invTrans1 = body1->GetInterpolationTransform().Inverse(), invTrans2 = body2->GetInterpolationTransform().Inverse();
	for (int i = 0; i < count; ++i)
	{
		contacts[i].localPointA = contacts[i].worldPos - obb1.pos;
		contacts[i].localPointB = contacts[i].worldPos - obb2.pos;
	}

	manifold.Update(contacts, count);

	delete[] contacts;
	ret = std::vector<glm::vec2>();
	quad.clear();
	rect.clear();
	return true;
}

void NarrowphaseSAT::CullPoints(std::vector<glm::vec2> points, int finalCount, int firstEntry, int culledPoints[])
{
	// Compute centroid
	size_t pointCount = points.size();
	glm::vec2 centroid(0, 0);
	float a, q;

	if (pointCount == 1) 
		centroid = points[0];
	else if (pointCount == 2) 
		centroid = 0.5f * (points[0] + points[1]);
	else 
	{
		a = 0;

		for (int i = 0; i < (pointCount - 1); i++) 
		{
			q = points[i].x * points[i + 1].y - points[i + 1].x * points[i].y;
			a += q;
			centroid += q * (points[i] + points[i + 1]);
		}

		q = points[pointCount - 1].x * points[0].y - points[0].x * points[pointCount - 1].y;
		a = 1.f/((3.f)*(a + q));
		centroid = a * (centroid + q * (points[pointCount - 1] + points[0]));
	}

	//Compute the angle of each point w.r.t. the centroid
	float angles[8];
	for (int i = 0; i < pointCount; i++) 
		angles[i] = std::atan2f(points[i].y - centroid.y, points[i].x - centroid.x);

	//Search for points that have angles closest to angles[firstEntry] + i*(2*pi/finalCount).
	int avail[8];
	for (int i = 0; i < pointCount; i++) 
		avail[i] = 1;

	avail[firstEntry] = 0;
	culledPoints[0] = firstEntry;
	culledPoints++;
	

	for (int j = 1; j < finalCount; j++) 
	{
		a = float(float(j)*(2 * M_PI / finalCount) + angles[firstEntry]);

		if (a > M_PI) 
			a -= (float)(2 * M_PI);

		float maxDiff = 1e9;
		float diff;
//#ifndef dNODEBUG
//		*iret = i0;			// iret is not allowed to keep this value
//#endif
		for (int i = 0; i < pointCount; i++) 
		{
			if (avail[i]) 
			{
				diff = std::fabs(angles[i] - a);
				if (diff > M_PI) 
					diff = (float)(2 * M_PI - diff);
				if (diff < maxDiff)
				{
					maxDiff = diff;
					*culledPoints = i;
				}
			}
		}
//#ifndef dNODEBUG
//		dIASSERT(*iret != i0);	// ensure iret got set
//#endif
		avail[*culledPoints] = 0;
		culledPoints++;
	}
}