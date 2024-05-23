#ifndef I_OBJECT_COMPONENT_H
#define I_OBJECT_COMPONENT_H

#include "../Utilities/OrbitalsDefs.h"
#include "ICelestialObject.h"

class IObjectComponent
{
public:
	IObjectComponent()
	{
		m_owner = 0;
		m_inUse = true;
	};
	virtual ~IObjectComponent() {};

	virtual void Update(float dt) {};

	void SetOwner(ICelestialObject* owner)
	{
		m_owner = owner;
		m_owner->AddComponent(std::shared_ptr<IObjectComponent>(this));
	};

	void SetInUse(bool inUse)
	{
		m_inUse = inUse;
	}
	bool GetInUse()
	{
		return m_inUse;
	}

	ICelestialObject* GetOwner() const { return m_owner;  };
	virtual const char* GetName() = 0;
	virtual ObjComponentID GetComponentID() = 0;
	virtual unsigned int GetID() { return m_id; }

protected:
	  ICelestialObject* m_owner;
	  bool m_inUse = false;
	  unsigned int m_id = 0;
};

#endif
