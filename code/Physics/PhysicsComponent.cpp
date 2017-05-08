#include "PhysicsComponent.h"

#include <glm/gtx/matrix_decompose.hpp>

#include "../Objects/ICelestialObject.h"

void PhysicsComponent::Update(float dt)
{
	//TODO: I'm not sure if this is the best way to handle movement
	/*glm::mat4 transform = m_rigidBody->GetTransform();

	glm::vec3 scale;
	glm::quat rotation;
	glm::vec3 translation;
	glm::vec3 skew;
	glm::vec4 perspective;
	glm::decompose(transform, scale, rotation, translation, skew, perspective);

	m_owner->SetPosition(translation);
	m_owner->SetRotation(glm::conjugate(rotation));*/
	m_owner->SetOpenGLMatrix(m_rigidBody->GetTransform());
}

void PhysicsComponent::SetBody(IRigidBody* body)
{
	m_rigidBody = body;
}

const const char* PhysicsComponent::COMPONENT_NAME = "PHYSICS_COMPONENT";
const ObjComponentID PhysicsComponent::COMPONENT_ID = Orbitals::HashedString::hash_name(PhysicsComponent::COMPONENT_NAME);