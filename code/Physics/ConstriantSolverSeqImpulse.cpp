#include "ConstraintSolverSeqImpulse.h"
#include "../Utilities/HelperFunctions.h"
#include "glm\gtc\matrix_transform.hpp"
#include "glm\gtx\norm.hpp"

void ConstraintSolverSeqImpulse::SolveConstraints(std::vector<PhysicsDefs::CollPairContactInfo>& info, float dt)
{
	glm::vec3 dv, impulse, impulseTangent, tangent, vel1, vel2, aVel1, aVel2, correction;;
	
	float massNormal, massTangent;
	glm::vec3 localANorm, localBNorm, localATang, localBTang;
	OTransform interpolationTrans1;
	OTransform interpolationTrans2;
	glm::mat3 invTensor1, invTensor2;

	PhysicsDefs::ContactInfo* contact;

	float invM1, invM2, totalInvMass, friction, minRest, invDt = 1.f/dt;
	float JV, JinvMJ;
	float lambda, oldLambda;
	float bias;
	glm::vec3 invMJ;
	glm::vec3 impulse1, impulse2, torque1, torque2, relativeVel;
	glm::vec3 startingVel1, startingVel2;

	for (int j = 0; j < 10; ++j)
	{
		for (int i = 0; i < info.size(); ++i) 
		{

			contact = &info[i].second;
			startingVel1 = info[i].first.first->GetLinearVelocity();
			startingVel2 = info[i].first.second->GetLinearVelocity();
			interpolationTrans1 = info[i].first.first->GetInterpolationTransform();
			interpolationTrans2 = info[i].first.second->GetInterpolationTransform();
			invM1 = info[i].first.first->GetInverseMass();
			invM2 = info[i].first.second->GetInverseMass();

			totalInvMass = invM1 + invM2;

			invTensor1 = info[i].first.first->GetInverseInertiaTensor();
			invTensor2 = info[i].first.second->GetInverseInertiaTensor();

			localANorm = glm::cross(contact->localPointA, contact->normal);
			localBNorm = glm::cross(contact->localPointB, contact->normal);

			relativeVel = vel2 - vel1;

			minRest = std::min(info[i].first.first->GetRestitution(), info[i].first.second->GetRestitution());

			JinvMJ = totalInvMass + glm::dot(localANorm * invTensor1, localANorm) +
					glm::dot(localBNorm * invTensor2, localBNorm);
			
			vel1 = info[i].first.first->GetLinearVelocity();
			vel2 = info[i].first.second->GetLinearVelocity();
			relativeVel = vel2 - vel1;
			aVel1 = info[i].first.first->GetAngularVelocity();
			aVel2 = info[i].first.second->GetAngularVelocity();

			JV = -glm::dot(vel1, contact->normal) - glm::dot(localANorm, aVel1)
				+ glm::dot(vel2, contact->normal) + glm::dot(localBNorm, aVel2);
			//bias = 0;
			bias = 0.3f / dt * std::max(0.f, contact->depth - 0.01f);
			auto test = minRest * JV;
			//bias -= 200 * std::abs(glm::dot(relativeVel, info[i].second.normal));//;
			//bias += test;
			//bias += test;
			/*lambda = (-JV + bias) / JinvMJ;

			{
				oldLambda = contact->prevNormalImp;
				contact->prevNormalImp = std::max(oldLambda + lambda, 0.f);
				lambda = contact->prevNormalImp - oldLambda;
			}*/

			//lambda -= bias / JinvMJ;
			impulse1 = contact->normal * lambda;
			impulse2 = contact->normal * lambda;

			info[i].first.first->ApplyImpulse(-impulse1);
			info[i].first.second->ApplyImpulse(impulse2);
			//info[i].first.first->SetAngularVelocity(aVel1 + invM1 * glm::cross(info[i].second.worldPointA, impulse));
			//info[i].first.second->SetAngularVelocity(aVel2 - invM2 * glm::cross(info[i].second.worldPointB, impulse));
			continue;

			vel1 = info[i].first.first->GetLinearVelocity();
			vel2 = info[i].first.second->GetLinearVelocity();

			//// Apply contact impulse
			//impulseTangent = dImpTang * tangent;
			info[i].first.first->ApplyImpulse(-impulse1);
			info[i].first.second->ApplyImpulse(impulse2);

			//info[i].first.first->SetAngularVelocity(aVel1 + invM1 * glm::cross(info[i].second.worldPointA, impulse));
			//info[i].first.second->SetAngularVelocity(aVel2 - invM2 * glm::cross(info[i].second.worldPointB, impulse));
		}
	}

}

void ConstraintSolverSeqImpulse::SolveConstraints2(std::vector<Manifold>& manifolds, float dt)
{
	PreStep(manifolds, dt);
	for (int j = 0; j < 10; ++j)
	{
		for (int i = 0; i < manifolds.size(); ++i)
		{
			for (int k = 0; k < manifolds[i].m_contactCount; ++k)
			{
				SolveContact(manifolds[i].m_bodyA, manifolds[i].m_bodyB, manifolds[i].m_contacts[k], dt);
			}
		}
	}
}

float CalcJV(const glm::vec3& normal,
	const glm::vec3& r1, const glm::vec3& lVel1, const glm::vec3& aVel1,
	const glm::vec3& r2, const glm::vec3& lVel2, const glm::vec3& aVel2
	)
{
	float JV;
	//glm::vec3 localANorm = ;
	//glm::vec3 localBNorm = glm::cross(r2, normal);
	//float lComp1 = glm::dot(normal, lVel1);
	//float lComp2 = glm::dot(normal, lVel2);
	float lComp1 = glm::dot(normal, lVel1);
	float lComp2 = glm::dot(normal, lVel2);
	//float aComp1 = 0;
	//float aComp2 = 0;
	//float aComp1 = glm::dot(aVel1, glm::cross(r1, normal));
	//float aComp2 = glm::dot(aVel2, glm::cross(r2, normal));
	float aComp1 = glm::dot(aVel1, glm::cross(normal, r1));
	float aComp2 = glm::dot(aVel2, glm::cross(normal, r2));


	JV = -lComp1 - aComp1 + lComp2 + aComp2;

	return JV;

}
void ConstraintSolverSeqImpulse::PreStep(std::vector<Manifold>& manifolds, float dt)
{
	PhysicsDefs::ContactInfo* contact;
	float invM1, invM2, totalInvMass, minRest, friction;
	glm::vec3 localANorm, localBNorm, localATangent, localBTangent, vel1, vel2, aVel1, aVel2;
	glm::mat3 invTensor1, invTensor2;
	float JV;
	for (int i = 0; i < manifolds.size(); ++i)
	{
		invM1 = manifolds[i].m_bodyA->GetInverseMass();
		invM2 = manifolds[i].m_bodyB->GetInverseMass();
		invTensor1 = manifolds[i].m_bodyA->GetInverseInertiaTensor();
		invTensor2 = manifolds[i].m_bodyB->GetInverseInertiaTensor();
		vel1 = manifolds[i].m_bodyA->GetLinearVelocity();
		vel2 = manifolds[i].m_bodyB->GetLinearVelocity();
		aVel1 = manifolds[i].m_bodyA->GetAngularVelocity();
		aVel2 = manifolds[i].m_bodyB->GetAngularVelocity();
		minRest = std::min(manifolds[i].m_bodyA->GetRestitution(), manifolds[i].m_bodyB->GetRestitution());
		friction = manifolds[i].m_bodyA->GetFriction() * manifolds[i].m_bodyB->GetFriction();
		totalInvMass = invM1 + invM2;
		for (int k = 0; k < manifolds[i].m_contactCount; ++k)
		{
			contact = &manifolds[i].m_contacts[k];
			//Normal Mass
			localANorm = glm::cross(contact->localPointA, contact->normal);
			localBNorm = glm::cross(contact->localPointB, contact->normal);
			contact->massNormal = 1.f/(totalInvMass + glm::dot(localANorm * invTensor1, localANorm) +
				glm::dot(localBNorm * invTensor2, localBNorm));

			////TODO: TANGENT
			//Based on Errin Catto's "Computing a Basis" article
			if (std::abs(contact->normal.x) >= 0.57735f)
				contact->tangent1 = glm::vec3(-contact->normal.y, contact->normal.x, 0.0f);
			else
				contact->tangent1 = glm::vec3(0.0f, contact->normal.z, -contact->normal.y);
			contact->tangent1 = glm::normalize(contact->tangent1);
			contact->tangent2 = glm::cross(contact->normal, contact->tangent1);

			//Mass tangent 1
			localATangent = glm::cross(contact->localPointA, contact->tangent1);
			localBTangent = glm::cross(contact->localPointB, contact->tangent1);
			contact->massTangent1 = 1.f / (totalInvMass + glm::dot(localATangent * invTensor1, localATangent) +
				glm::dot(localBTangent * invTensor2, localBTangent));
			
			//Mass tangent 2
			localATangent = glm::cross(contact->localPointA, contact->tangent2);
			localBTangent = glm::cross(contact->localPointB, contact->tangent2);
			contact->massTangent2 = 1.f / (totalInvMass + glm::dot(localATangent * invTensor1, localATangent) +
				glm::dot(localBTangent * invTensor2, localBTangent));

			//Bias
			JV = CalcJV(contact->normal, contact->localPointA, vel1, aVel1, contact->localPointB, vel2, aVel2);
			contact->bias = -0.1f / dt * std::max(0.0f, contact->depth -0.001f) + minRest * JV;
			contact->friction = friction;
		}
	}
}

void ConstraintSolverSeqImpulse::SolveContact(IRigidBody* body1, IRigidBody* body2, PhysicsDefs::ContactInfo& contact, float dt)
{
	//TODO: FIND ERROR
	//glm::vec3 normal = contact.normal;

	glm::vec3 dv, impulse, impulseTangent, tangent, vel1, vel2, aVel1, aVel2;
	//body1->SetLinearVelocity(glm::vec3(0.f));
	//body2->SetLinearVelocity(glm::vec3(0.f));
	
	float massNormal, massTangent, invM1, invM2, f;
	glm::vec3 localANorm, localBNorm, localATang, localBTang;
	//glm::mat4 interpolationTrans1;
	//glm::mat4 interpolationTrans2;

	float friction, invDt = 1.f / dt;
	float JV;
	float lambda, oldLambda;

	glm::vec3 invMJ;
	glm::vec3 impulse1, impulse2, torque1, torque2;

	invM1 = body1->GetInverseMass();
	invM2 = body1->GetInverseMass();
	//localANorm = glm::cross(contact.localPointA, normal);
	//localBNorm = glm::cross(contact.localPointB, normal);

	vel1 = body1->GetLinearVelocity();
	vel2 = body2->GetLinearVelocity();
	//relativeVel = vel2 - vel1;
	aVel1 = body1->GetAngularVelocity();
	aVel2 = body2->GetAngularVelocity();
	localANorm = glm::cross(contact.localPointA, aVel1);
	localBNorm = glm::cross(contact.localPointB, aVel2);
	JV = CalcJV(contact.normal, contact.localPointA, vel1, aVel1, contact.localPointB, vel2, aVel2);

	lambda = -(JV + contact.bias) * contact.massNormal;

	{
		oldLambda = contact.prevNormalImp;
		contact.prevNormalImp = std::max(oldLambda + lambda, 0.f);
		lambda = contact.prevNormalImp - oldLambda;
	}


	impulse = contact.normal * lambda;
	torque1 = glm::cross(contact.localPointA, impulse);
	torque2 = glm::cross(contact.localPointB, impulse);
	body1->ApplyImpulse(-impulse);
	body2->ApplyImpulse(impulse);
	body1->ApplyTorqueImpulse(torque1);
	body2->ApplyTorqueImpulse(-torque2);
	
	float coeff = contact.prevNormalImp * contact.friction;
	vel1 = body1->GetLinearVelocity();
	vel2 = body2->GetLinearVelocity();
	//relativeVel = vel2 - vel1;
	aVel1 = body1->GetAngularVelocity();
	aVel2 = body2->GetAngularVelocity();
	//Apply friction for tangent1
	JV = -glm::dot(vel1, contact.tangent1) - glm::dot(localANorm, contact.tangent1)
		+ glm::dot(vel2, contact.tangent1) + glm::dot(localBNorm, contact.tangent1);
	lambda = -JV * contact.massTangent1;
	{
		oldLambda = contact.prevTangImp1;
		contact.prevTangImp1 = Clamp(oldLambda + lambda, -coeff, coeff);
		lambda = contact.prevTangImp1 - oldLambda;
	}
	impulse = contact.tangent1 * lambda;
	torque1 = glm::cross(contact.localPointA, impulse);
	torque2 = glm::cross(contact.localPointB, impulse);
	body1->ApplyImpulse(-impulse);
	body2->ApplyImpulse(impulse);
	body1->ApplyTorqueImpulse(torque1);
	body2->ApplyTorqueImpulse(-torque2);

	//Apply friction for tangent2
	//vel1 = body1->GetLinearVelocity();
	//vel2 = body2->GetLinearVelocity();
	////relativeVel = vel2 - vel1;
	//aVel1 = body1->GetAngularVelocity();
	//aVel2 = body2->GetAngularVelocity();
	JV = -glm::dot(vel1, contact.tangent2) - glm::dot(localANorm, contact.tangent2)
		+ glm::dot(vel2, contact.tangent2) + glm::dot(localBNorm, contact.tangent2);
	lambda = -JV * contact.massTangent2;
	{
		oldLambda = contact.prevTangImp2;
		contact.prevTangImp2 = Clamp(oldLambda + lambda, -coeff, coeff);
		lambda = contact.prevTangImp2 - oldLambda;
	}
	impulse = contact.tangent2 * lambda;
	torque1 = glm::cross(contact.localPointA, impulse);
	torque2 = glm::cross(contact.localPointB, impulse);
	body1->ApplyImpulse(-impulse);
	body2->ApplyImpulse(impulse);
	body1->ApplyTorqueImpulse(torque1);
	body2->ApplyTorqueImpulse(-torque2);
}


//Only contact constraint for now
//float xDepth, yDepth, zDepth;
//glm::vec3 vel1, vel2, relativeVel, impulse, correction;
//glm::mat4 invTransform1;
//glm::mat4 interpolationTrans1;
//glm::mat4 interpolationTrans2;
//float velAlongColNormal, minConst, totalSytemMass, mass1, mass2, invMass1, invMass2;
//interpolationTrans1 = info[i].first.first->GetInterpolationTransform();
//interpolationTrans2 = info[i].first.second->GetInterpolationTransform();
//
//
//relativeVel = vel2 - vel1;
//info[i].second.tangent1 = glm::normalize(relativeVel);
//info[i].second.tangent2 = glm::cross(info[i].second.tangent1, info[i].second.normal);
//velAlongColNormal = glm::dot(relativeVel, info[i].second.normal);
//if (velAlongColNormal < 0)
//	continue;
//minConst = std::min(info[i].first.first->GetRestitution(), info[i].first.second->GetRestitution());
//mass1 = info[i].first.first->GetMass();
//invMass1 = info[i].first.first->GetInverseMass();
//mass2 = info[i].first.second->GetMass();
//invMass2 = info[i].first.second->GetInverseMass();
//totalSytemMass = mass1 + mass2;
//impulse = info[i].second.normal * (-(1 + minConst) * velAlongColNormal);
//
//
////Pushout to avoid sinking
//const float PERCENT = 1;
//const float THRESHOLD = 0.001;
//correction = std::max(info[i].second.depth - THRESHOLD, 0.0f) / (invMass1 + invMass2) * PERCENT * info[i].second.normal;
//interpolationTrans1 = glm::translate(interpolationTrans1, invMass1 * correction);
//interpolationTrans2 = glm::translate(interpolationTrans2, -invMass2 * correction);
//info[i].first.first->UpdateInterpolationTransform(interpolationTrans1);
//info[i].first.second->UpdateInterpolationTransform(interpolationTrans2);
////Apply friction
//float frict1 = info[i].first.first->GetFriction();
//float frict2 = info[i].first.second->GetFriction();
////collisionPairs[i].first->SetLinearVelocity(glm::vec3(0, 0, 0));
////collisionPairs[i].second->SetLinearVelocity(glm::vec3(0, 0, 0));