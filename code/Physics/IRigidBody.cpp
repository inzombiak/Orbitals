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
	m_friction			= rbci.friction;
	m_rollingFriction	= rbci.rollingFriction;
	m_restitution		= rbci.resititution;

	m_transform			= rbci.transform;
	m_localInertia		= rbci.localInertia;
	m_collisionShape	= rbci.collisionShape;
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
	m_linearVelocity += impulse * m_invMass;
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

void IRigidBody::GetAABB(glm::vec3& aabbMin, glm::vec3& aabbMax)
{
	//TODO Add assert
	if (m_collisionShape)
	{
		m_collisionShape->GetAABB(aabbMin, aabbMax);
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
}
glm::vec3 IRigidBody::GetAngularVelocity() const
{
	return m_angularVelocity;
}
float IRigidBody::GetMass() const
{
	return m_mass;
}
float IRigidBody::GetInverseMass() const
{
	return m_invMass;
}
void IRigidBody::UpdateTransform(const glm::mat4& predictedTrans)
{
	m_transform = predictedTrans;
}
const glm::mat4& IRigidBody::GetTransform() const
{
	return m_transform;
}
glm::mat4& IRigidBody::GetInterpolationTransform()
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