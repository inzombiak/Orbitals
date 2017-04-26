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

	PhysicsDefs::AABB& GetAABB();

	glm::vec3 GetTotalForce() const;
	glm::vec3 GetLinearVelocity() const;
	void SetLinearVelocity(const glm::vec3& newLinVel);
	glm::vec3 GetAngularVelocity() const;
	float GetMass() const;
	float GetInverseMass() const;

	void UpdateTransform(const glm::mat4& predictedTrans);
	const glm::mat4& GetTransform() const;
	void UpdateInterpolationTransform(const glm::mat4& predictedTrans);
	const glm::mat4& GetInterpolationTransform();

	float GetFriction() const;
	float GetRollingFriction() const;
	float GetRestitution() const;

	//For GJK and EPA
	glm::vec3 GetSupportPoint(glm::vec3 dir) const;

private:

	glm::vec3 m_gravity;

	float m_mass;
	float m_invMass;
	
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

	PhysicsDefs::AABB m_aabb;

	ICollisionShape* m_collisionShape;
};

#endif