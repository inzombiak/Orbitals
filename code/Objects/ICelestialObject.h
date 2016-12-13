#ifndef I_CELESTIAL_OBJECT_H
#define I_CELESTIAL_OBJECT_H

#include "../Utilities/OrbitalsDefs.h"
#include <unordered_map>

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <string>


class ICelestialObject
{
public:
	ICelestialObject(const std::string name, unsigned int id, CelestialObjTypes type) : m_name(name), m_id(id), m_type(type)
	{
		m_position = glm::vec3(0, 0, 0);
		m_scale = glm::vec3(1, 1, 1);
		m_rotation = glm::angleAxis(0.f, glm::vec3(1, 1, 1));
	};
	virtual ~ICelestialObject(){};
	
	WeakObjComponentPtr GetComponent(ObjComponentID compType);
	bool AddComponent(StrongObjComponentPtr component);

	void SetPosition(const glm::vec3& pos);
	glm::vec3 GetPosition() const;

	void SetRotation(const glm::quat& rotation);
	glm::quat GetRotation() const;
	
	void SetScale(const glm::vec3& scale);
	glm::vec3 GetScale();

	glm::mat4 GetOpenGLMatrix() const;

	std::string GetName() const
	{
		return m_name;
	}
	int GetID() const
	{
		return m_id;
	}
	
private:

	Orbitals::CelestialObjTypes m_type;
	Orbitals::CelestialObjID m_id;

	glm::vec3 m_position;
	glm::quat m_rotation;
	glm::vec3 m_scale;

	std::string m_name;
	typedef std::unordered_map<ObjComponentID, StrongObjComponentPtr, std::hash<int>> ComponentMap;
	ComponentMap m_components;

	template <class ObjComponent>
	std::weak_ptr<ObjComponent> GetActorComponentByID(ObjComponentID componentID)
	{
		//Check if enitiy contains the component
		EntityComponents::iterator componentIterator = m_components.find(componentID);
		if (componentIterator != m_components.end())
		{
			//Get the component, cast it to the derived type, store it in weak pointer and return
			StrongObjComponentPtr pBase(componentIterator->second);
			weak_ptr<ObjComponent> pDeriv(static_pointer_cast(pBase));
			return pDeriv;
		}
		else
		{
			//Return empty pointer if no such component is found
			return weak_ptr<ObjComponent>();
		}
	}
};

#endif