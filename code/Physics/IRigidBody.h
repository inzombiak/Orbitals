#ifndef I_RIGID_BODY_H
#define I_RIGID_BODY_H

#include "ICollisionShape.h"

enum RigidBodyType
{
	Static = 1,
	Dynamic = 2,
};

struct RigidBodyConstructionInfo
{
	float mass;
	float linearDamping;
	float angularDamping;
	float friction;
	float rollingFriction;
	float resititution;

	glm::mat4 transform;
	glm::vec3 localInertia;

	ICollisionShape* collisionShape;
};

class IRigidBody
{
public:
	IRigidBody(const RigidBodyConstructionInfo& rbci);
	virtual ~IRigidBody() {};

	void ClearForces();

	void ApplyImpulse(const glm::vec3& impulse);
	void ApplyForce(const glm::vec3& force);
	void ApplyDamping(float timeStep);

	void GetAABB(glm::vec3& aabbMin, glm::vec3& aabbMax);

	glm::vec3 GetTotalForce() const;
	glm::vec3 GetLinearVelocity() const;
	glm::vec3 GetAngularVelocity() const;

	void UpdateTransform(const glm::mat4& predictedTrans);
	const glm::mat4& GetTransform() const;

private:

	float m_mass;
	
	glm::vec3 m_linearVelocity;
	glm::vec3 m_angularVelocity;

	glm::mat4 m_transform;
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