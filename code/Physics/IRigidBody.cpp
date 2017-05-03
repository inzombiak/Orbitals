#include "IRigidBody.h"

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
	m_friction			= rbci.friction - 0.9;
	m_rollingFriction	= rbci.rollingFriction;
	m_restitution		= rbci.resititution;

	m_transform			= rbci.transform;
	m_localInertia		= rbci.localInertia;
	m_collisionShape	= rbci.collisionShape;

	m_aabb = m_collisionShape->GetAABB();
	m_aabb.body = this;
	m_aabb.worldTransfrom = m_transform;
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
	m_totalForce += m_gravity;
}
void IRigidBody::ApplyImpulse(const glm::vec3& impulse)
{
	SetLinearVelocity(m_linearVelocity + impulse * m_invMass);
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
	//TODO Add assert
	if (m_collisionShape)
	{
		return m_aabb;
	}
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
}
glm::vec3 IRigidBody::GetAngularVelocity() const
{
	return m_angularVelocity;
}

void IRigidBody::SetAngularVelocity(const glm::vec3& newAngVel)
{
	m_angularVelocity = newAngVel;
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
void IRigidBody::UpdateTransform(const glm::mat4& predictedTrans)
{
	m_transform = predictedTrans;
	m_aabb.worldTransfrom = m_transform;
}
const glm::mat4& IRigidBody::GetTransform() const
{
	return m_transform;
}
void IRigidBody::UpdateInterpolationTransform(const glm::mat4& predictedTrans)
{
	m_aabb.worldTransfrom = predictedTrans;
	m_interpolationTransform = predictedTrans;
}
const glm::mat4& IRigidBody::GetInterpolationTransform()
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