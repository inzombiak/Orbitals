#include "IRigidBody.h"

#include <glm\gtc\matrix_transform.hpp>

IRigidBody::IRigidBody(const PhysicsDefs::RigidBodyConstructionInfo& rbci)
{
	m_gravity = glm::vec3(0.f);

	m_mass				= rbci.mass;
	if (m_mass == 0)
		m_invMass		= 0;
	else
		m_invMass		= 1 / m_mass;
	m_linearDamping		= rbci.linearDamping;
	m_angularDamping	= rbci.angularDamping;
	m_friction			= rbci.friction;
	m_rollingFriction	= rbci.rollingFriction;
	m_restitution		= rbci.resititution;

	m_transform			= rbci.transform;
	m_localInertia		= rbci.localInertia;
	m_collisionShape	= rbci.collisionShape;

	m_enableGravity		= rbci.enableGravity;

	m_obb = m_collisionShape->GetLocalOBB();
	m_obb.aabb.body = this;
	UpdateInterpolationTransform(m_transform);

	if (m_mass == 0)
		m_invInertiaTensor = glm::mat3(0.f);
	else
		m_invInertiaTensor = glm::inverse(m_collisionShape->GetTensor(m_mass));
}

void IRigidBody::ClearForces()
{
	m_totalForce = glm::vec3(0);
}
void IRigidBody::ApplyGravity()
{
	if (!m_enableGravity)
		return;
	m_totalForce += m_gravity;
}
void IRigidBody::ApplyImpulse(const glm::vec3& impulse)
{
	SetLinearVelocity(m_linearVelocity + impulse * m_invMass);
}
void IRigidBody::ApplyTorqueImpulse(const glm::vec3& torque)
{
	SetAngularVelocity(m_angularVelocity + m_invInertiaTensor * torque);

}

void IRigidBody::ApplyForce(const glm::vec3& force)
{
	m_totalForce += force;
}
void IRigidBody::ApplyDamping(float timeStep)
{
	m_linearVelocity	*= std::powf(1 - m_linearDamping, timeStep);
	m_angularVelocity	*= std::powf(1 - m_angularDamping, timeStep);
}

void IRigidBody::SetGravity(const glm::vec3& gravity)
{
	if (m_mass != 0)
	{
		m_gravity = gravity * m_mass;
	}
}

PhysicsDefs::AABB& IRigidBody::GetAABB()
{
	return m_obb.aabb;
}

PhysicsDefs::OBB& IRigidBody::GetOBB()
{
	return m_obb;
}

glm::vec3 IRigidBody::GetTotalForce() const
{
	return m_totalForce;
}
glm::vec3 IRigidBody::GetLinearVelocity() const
{
	return m_linearVelocity;
}
void IRigidBody::SetLinearVelocity(const glm::vec3& newLinVel)
{
	m_linearVelocity = newLinVel;
	if (glm::dot(m_linearVelocity, m_linearVelocity) < 0.01f)
		m_linearVelocity = glm::vec3(0);
	//else if (glm::dot(m_linearVelocity, m_linearVelocity) > 1000.f)
	//{
	//	float fraction = glm::dot(m_linearVelocity, m_linearVelocity) / 1000.f;
	//	m_linearVelocity /= fraction;
	//}
}
glm::vec3 IRigidBody::GetAngularVelocity() const
{
	return m_angularVelocity;
}

void IRigidBody::SetAngularVelocity(const glm::vec3& newAngVel)
{
	m_angularVelocity = newAngVel;

	if (glm::dot(m_angularVelocity, m_angularVelocity) < 0.000001f)
		m_angularVelocity = glm::vec3(0);
}
float IRigidBody::GetMass() const
{
	return m_mass;
}
float IRigidBody::GetInverseMass() const
{
	return m_invMass;
}
glm::mat3  IRigidBody::GetInverseInertiaTensor() const
{
	return m_invInertiaTensor;
}
void IRigidBody::UpdateTransform(const OTransform& predictedTrans)
{
	m_transform = predictedTrans;
	UpdateInterpolationTransform(m_transform);
}
const OTransform& IRigidBody::GetTransform() const
{
	return m_transform;
}
void IRigidBody::UpdateInterpolationTransform(const OTransform& predictedTrans)
{
	m_interpolationTransform = predictedTrans;
	//glm::mat4 tempMat;
	//tempMat = glm::translate(m_interpolationTransform.GetBasis(), glm::vec3(0, 0, 0));
	glm::mat3 rot = glm::transpose(m_interpolationTransform.GetBasis());
	//m_obb.localAxes[0] = glm::vec3(rot * glm::vec3(1, 0, 0));
	//m_obb.localAxes[1] = glm::vec3(rot * glm::vec3(0, 1, 0));
	//if (glm::dot(m_obb.localAxes[0], m_obb.localAxes[1]) <= 0)
	//	m_obb.localAxes[2] = glm::cross(m_obb.localAxes[0], m_obb.localAxes[1]);
	//else
	//	m_obb.localAxes[2] = glm::cross(m_obb.localAxes[1], m_obb.localAxes[0]);
	m_obb.localAxes[0] = rot[0];
	m_obb.localAxes[1] = rot[1];
	m_obb.localAxes[2] = rot[2];
	m_obb.pos = m_interpolationTransform.GetOrigin();
	m_obb.UpdateAABB();
	m_obb.aabb.worldTransform = glm::translate(glm::mat4(1.f), m_obb.pos);
}
const OTransform& IRigidBody::GetInterpolationTransform()
{
	return m_interpolationTransform;
}
float IRigidBody::GetFriction() const
{
	return m_friction;
}
float IRigidBody::GetRollingFriction() const
{
	return m_rollingFriction;
}
float IRigidBody::GetRestitution() const
{
	return m_restitution;
}
glm::vec3 IRigidBody::GetSupportPoint(glm::vec3 dir) const
{
	return m_collisionShape->GetSupportPoint(dir);
}