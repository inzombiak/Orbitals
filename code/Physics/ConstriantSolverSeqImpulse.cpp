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

	float m1, m2, invM1, invM2, totalInvMass, friction, minRest;
	float JV, JinvMJ;
	float lambda, oldLambda;
	float bias;
	glm::vec3 invMJ;
	glm::vec3 impulse1, impulse2, torque1, torque2, relativeVel;

	for (int i = 0; i < 10; ++i)
	{ 
		for (int i = 0; i < info.size(); ++i)
		{
			interpolationTrans1 = info[i].first.first->GetInterpolationTransform();;
			interpolationTrans2 = info[i].first.second->GetInterpolationTransform();

			vel1 = info[i].first.first->GetLinearVelocity();
			vel2 = info[i].first.second->GetLinearVelocity();

			aVel1 = info[i].first.first->GetAngularVelocity();
			aVel2 = info[i].first.second->GetAngularVelocity();

			invM1 = info[i].first.first->GetInverseMass();
			invM2 = info[i].first.second->GetInverseMass();

			m1 = info[i].first.first->GetMass();
			m2 = info[i].first.second->GetMass();

			totalInvMass = invM1 + invM2;

			invTensor1 = info[i].first.first->GetInverseInertiaTensor();
			invTensor2 = info[i].first.second->GetInverseInertiaTensor();

			localANorm = glm::cross(info[i].second.localPointA, info[i].second.normal);
			localBNorm = glm::cross(info[i].second.localPointB, info[i].second.normal);

			JV = -glm::dot(vel1, info[i].second.normal) - glm::dot(localANorm, aVel1)
				+ glm::dot(vel2, info[i].second.normal) + glm::dot(localBNorm, aVel2);
			
			JinvMJ = totalInvMass + glm::dot(localANorm * invTensor1, localANorm) +
				  glm::dot(localBNorm * invTensor2, localBNorm);

			bias = -0.4f / dt * std::max(0.f, info[i].second.depth - 0.001f);
			minRest = std::min(info[i].first.first->GetRestitution(), info[i].first.second->GetRestitution());
			relativeVel = vel2 - vel1;
			bias += minRest * glm::dot(relativeVel, info[i].second.normal);

			lambda = -(JV + bias) / JinvMJ;

			{
				oldLambda = info[i].second.prevNormalImp;
				info[i].second.prevNormalImp = std::max(info[i].second.prevNormalImp + lambda, 0.f);

				lambda = info[i].second.prevNormalImp - oldLambda;
			}

			impulse1 = info[i].second.normal * lambda;
			impulse2 = info[i].second.normal * lambda;

			info[i].first.first->ApplyImpulse(-impulse1);
			info[i].first.second->ApplyImpulse(impulse2);
			//info[i].first.first->SetAngularVelocity(aVel1 + invM1 * glm::cross(info[i].second.worldPointA, impulse));
			//info[i].first.second->SetAngularVelocity(aVel2 - invM2 * glm::cross(info[i].second.worldPointB, impulse));
			continue;

			vel1 = info[i].first.first->GetLinearVelocity();
			vel2 = info[i].first.second->GetLinearVelocity();

			//dv = vel2 + glm::cross(aVel2, info[i].second.localPointB) -
			//	vel1 - glm::cross(aVel1, info[i].second.localPointA);
			//friction = info[i].first.first->GetFriction()*info[i].first.second->GetFriction();
			//tangent = glm::cross(info[i].second.normal, glm::vec3(1.0f));
			//localATang = glm::cross(info[i].second.localPointA, tangent);
			//localBTang = glm::cross(info[i].second.localPointB, tangent);
			//massTangent = totalInvMass + glm::dot(localATang * invTensor1, localATang) +
			//							 glm::dot(localBTang * invTensor2, localBTang);

			////massTangent = totalInvMass + (glm::dot(info[i].second.localPointA, info[i].second.localPointA) - localATang*localATang) +
			////	(glm::dot(info[i].second.localPointB, info[i].second.localPointB) - localBTang*localBTang);
			////massTangent = 1.f / massTangent;
			//massTangent = 1;
			//impTang = glm::dot(dv, tangent);
			//dImpTang = massTangent * -impTang;

			//float maxPt = friction * info[i].second.prevNormalImp;
			//float oldTangentImpulse = info[i].second.prevTangImp;
			//info[i].second.prevTangImp = std::max(-maxPt, std::min(oldTangentImpulse + dImpTang, maxPt));
			//dImpTang = info[i].second.prevTangImp - oldTangentImpulse;

			//// Apply contact impulse
			//impulseTangent = dImpTang * tangent;
			info[i].first.first->ApplyImpulse(-impulse1);
			info[i].first.second->ApplyImpulse(impulse2);

			//info[i].first.first->SetAngularVelocity(aVel1 + invM1 * glm::cross(info[i].second.worldPointA, impulse));
			//info[i].first.second->SetAngularVelocity(aVel2 - invM2 * glm::cross(info[i].second.worldPointB, impulse));
		}
	}

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