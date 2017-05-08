#include "ICelestialObject.h"
#include "IObjectComponent.h"

#include <glm/gtc/matrix_transform.hpp>

void ICelestialObject::SetPosition(const glm::vec3& pos)
{
	m_position = pos;
}
glm::vec3 ICelestialObject::GetPosition() const
{
	return m_position;
}

void ICelestialObject::SetRotation(const glm::quat& rot)
{
	m_rotation = rot;
}
glm::quat ICelestialObject::GetRotation() const
{
	return m_rotation;
}

glm::mat4 ICelestialObject::GetOpenGLMatrix() const
{
	//printf("%d, %s:: OGLM pos %lf, %lf, %lf \n", __LINE__, __FILE__, m_position.x, m_position.y, m_position.z);

	/*glm::mat4 rotationMatrix = glm::toMat4(m_rotation);
	glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), m_position);
	glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), m_scale);
	
	openGLMat = translationMatrix * rotationMatrix * scaleMatrix;;*/

	return m_openGLMat;
}
void ICelestialObject::SetOpenGLMatrix(const glm::mat4 newMat)
{
	m_openGLMat = newMat;
}

WeakObjComponentPtr ICelestialObject::GetComponent(ObjComponentID compType)
{
	ComponentMap::iterator findComp = m_components.find(compType);

	if (findComp != m_components.end())
	{
		return WeakObjComponentPtr(findComp->second);
	}
	else
	{
		return WeakObjComponentPtr();
	}
}

bool ICelestialObject::AddComponent(StrongObjComponentPtr component)
{
	ObjComponentID compID = component->GetComponentID();

	ComponentMap::iterator findComp = m_components.find(compID);

	if (findComp == m_components.end())
	{
		m_components.insert(std::pair<ObjComponentID, StrongObjComponentPtr>(compID, component));

		return true;
	}
	else
	{
		return false;
	}
}