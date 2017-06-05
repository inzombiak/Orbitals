#include "ConstraintSolverSeqImpulse.h"

#include "glm\gtc\matrix_transform.hpp"
#include "glm\gtx\norm.hpp"

void ConstraintSolverSeqImpulse::SolveConstraints(std::vector<PhysicsDefs::CollPairContactInfo>& info, float dt)
{
	glm::vec3 dv, impulse, impulseTangent, tangent, vel1, vel2, aVel1, aVel2, correction;;
	
	float massNormal, massTangent;
	glm::vec3 localANorm, localBNorm, localATang, localBTang;
	glm::mat4 interpolationTrans1;
	glm::mat4 interpolationTrans2;
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
			lambda = (-JV + bias) / JinvMJ;

			{
				oldLambda = contact->prevNormalImp;
				contact->prevNormalImp = std::max(oldLambda + lambda, 0.f);
				lambda = contact->prevNormalImp - oldLambda;
			}

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

void ConstraintSolverSeqImpulse::PreStep(std::vector<Manifold>& manifolds, float dt)
{
	PhysicsDefs::ContactInfo* contact;
	float invM1, invM2, totalInvMass, minRest;
	glm::vec3 localANorm, localBNorm, vel1, vel2, aVel1, aVel2;
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
		totalInvMass = invM1 + invM2;
		for (int k = 0; k < manifolds[i].m_contactCount; ++k)
		{
			contact = &manifolds[i].m_contacts[k];
			//Normal Mass
			localANorm = glm::cross(contact->localPointA, contact->normal);
			localBNorm = glm::cross(contact->localPointB, contact->normal);
			JV = -glm::dot(vel1, contact->normal) - glm::dot(localANorm, aVel1)
				+ glm::dot(vel2, contact->normal) + glm::dot(localBNorm, aVel2);
			contact->massNormal = 1/(totalInvMass + glm::dot(localANorm * invTensor1, localANorm) +
				glm::dot(localBNorm * invTensor2, localBNorm));

			//TODO: TANGENT

			//Bias
			contact->bias = -0.1f / dt * std::max(0.0f, contact->depth - 0.001f) - minRest * JV;
		}
	}
}

void ConstraintSolverSeqImpulse::SolveContact(IRigidBody* body1, IRigidBody* body2, PhysicsDefs::ContactInfo& contact, float dt)
{
	//TODO: FIND ERROR
	glm::vec3 normal = contact.normal;

	glm::vec3 dv, impulse, impulseTangent, tangent, vel1, vel2, aVel1, aVel2, correction;;

	float massNormal, massTangent, invM1, invM2;
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
	localANorm = glm::cross(contact.localPointA, normal);
	localBNorm = glm::cross(contact.localPointB, normal);

	vel1 = body1->GetLinearVelocity();
	vel2 = body2->GetLinearVelocity();
	//relativeVel = vel2 - vel1;
	aVel1 = body1->GetAngularVelocity();
	aVel2 = body2->GetAngularVelocity();

	//auto bb = ()
	JV = -glm::dot(vel1, normal) - glm::dot(localANorm, aVel1)
		+ glm::dot(vel2, normal) + glm::dot(localBNorm, aVel2);

	//auto restitution = minRest * JV;
	lambda = (-JV + contact.bias) * contact.massNormal;
	/*{
		oldLambda = contact.prevNormalImp;
		contact.prevNormalImp = std::max(oldLambda + lambda, 0.f);
		lambda = contact.prevNormalImp - oldLambda;
	}*/

	impulse = normal * lambda;
	torque1 = glm::cross(contact.localPointA, impulse);
	torque2 = glm::cross(contact.localPointB, impulse);

	body1->ApplyImpulse(-impulse);
	body2->ApplyImpulse(impulse);
	//body1->SetLinearVelocity(glm::vec3(0.f));
	//body2->SetLinearVelocity(glm::vec3(0.f));
	//body1->ApplyTorqueImpulse(torque1);
	//body2->ApplyTorqueImpulse(-torque2);
	//body1->SetAngularVelocity(aVel1 + invM1 * glm::cross(contact.localPointA, impulse));
	//body2->SetAngularVelocity(aVel2 - invM2 * glm::cross(contact.localPointB, impulse));
	//// Apply contact impulse
	//impulseTangent = dImpTang * tangent;
	//body1->ApplyImpulse(-impulse1);
	//body2->ApplyImpulse(impulse2);

	//info[i].first.first->SetAngularVelocity(aVel1 + invM1 * glm::cross(info[i].second.worldPointA, impulse));
	//info[i].first.second->SetAngularVelocity(aVel2 - invM2 * glm::cross(info[i].second.worldPointB, impulse));
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