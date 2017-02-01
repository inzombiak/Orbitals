#ifndef I_RIGID_BODY_H
#define I_RIGID_BODY_H

#include "ICollisionShape.h"


class IRigidBody
{
public:
	IRigidBody(const PhysicsDefs::RigidBodyConstructionInfo& rbci);
	virtual ~IRigidBody() {};

	void ClearForces();

	void ApplyGravity();
	void ApplyImpulse(const glm::vec3& impulse);
	void ApplyForce(const glm::vec3& force);
	void ApplyDamping(float timeStep);

	void SetGravity(const glm::vec3& m_gravity);

	void GetAABB(glm::vec3& aabbMin, glm::vec3& aabbMax);

	glm::vec3 GetTotalForce() const;
	glm::vec3 GetLinearVelocity() const;
	void SetLinearVelocity(const glm::vec3& newLinVel);
	glm::vec3 GetAngularVelocity() const;
	float GetMass() const;

	void UpdateTransform(const glm::mat4& predictedTrans);
	const glm::mat4& GetTransform() const;
	glm::mat4& GetInterpolationTransform();

private:

	glm::vec3 m_gravity;

	float m_mass;
	
	glm::vec3 m_linearVelocity = glm::vec3(0.f);
	glm::vec3 m_angularVelocity = glm::vec3(0.f);

	glm::mat4 m_transform;
	glm::mat4 m_interpolationTransform;
	glm::vec3 m_localInertia;

	float m_linearDamping;
	float m_angularDamping;

	float m_friction;
	float m_rollingFriction;
	float m_restitution;

	glm::vec3 m_totalForce;

	ICollisionShape* m_collisionShape;
};

#endif