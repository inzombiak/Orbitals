#include "IRigidBody.h"

IRigidBody::IRigidBody(const RigidBodyConstructionInfo& rbci)
{
	m_mass				= rbci.mass;
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

void IRigidBody::ApplyImpulse(const glm::vec3& impulse)
{
	m_linearVelocity += impulse / m_mass;
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

void IRigidBody::GetAABB(glm::vec3& aabbMin, glm::vec3& aabbMax)
{
	//TODO Add assert
	if (m_collisionShape)
	{
		m_collisionShape->GetAABB(m_transform, aabbMin, aabbMax);
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
glm::vec3 IRigidBody::GetAngularVelocity() const
{
	return m_angularVelocity;
}

void IRigidBody::UpdateTransform(const glm::mat4& predictedTrans)
{
	m_transform = predictedTrans;
}
const glm::mat4& IRigidBody::GetTransform() const
{
	return m_transform;
}
