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
	void ApplyTorqueImpulse(const glm::vec3& torque);
	void ApplyForce(const glm::vec3& force);
	void ApplyDamping(float timeStep);

	void SetGravity(const glm::vec3& m_gravity);

	PhysicsDefs::AABB& GetAABB();
	PhysicsDefs::OBB& GetOBB();

	glm::vec3 GetTotalForce() const;
	glm::vec3 GetLinearVelocity() const;
	void SetLinearVelocity(const glm::vec3& newLinVel);
	glm::vec3 GetAngularVelocity() const;
	void SetAngularVelocity(const glm::vec3& newAngVel);
	float GetMass() const;
	float GetInverseMass() const;
	glm::mat3 GetInverseInertiaTensor() const;

	void UpdateTransform(const OTransform& predictedTrans);
	const OTransform& GetTransform() const;
	void UpdateInterpolationTransform(const OTransform& predictedTrans);
	const OTransform& GetInterpolationTransform();

	float GetFriction() const;
	float GetRollingFriction() const;
	float GetRestitution() const;

	//For GJK and EPA
	glm::vec3 GetSupportPoint(glm::vec3 dir) const;

private:
	bool m_enableGravity;
	glm::vec3 m_gravity;

	float m_mass;
	float m_invMass;
	
	glm::vec3 m_linearVelocity = glm::vec3(0.f);
	glm::vec3 m_angularVelocity = glm::vec3(0.f);

	//glm::mat4 m_transform;
	//glm::mat4 m_interpolationTransform;
	OTransform m_transform;
	OTransform m_interpolationTransform;
	glm::mat3 m_invInertiaTensor;
	glm::vec3 m_localInertia;

	float m_linearDamping;
	float m_angularDamping;

	float m_friction;
	float m_rollingFriction;
	float m_restitution;

	glm::vec3 m_totalForce;

	PhysicsDefs::OBB m_obb;
	ICollisionShape* m_collisionShape;
};

#endif