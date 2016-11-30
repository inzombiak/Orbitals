#ifndef OBJECT_MANAGER_H
#define OBJECT_MANAGER_H
#include "ISystem.h"
#include "ICelestialObject.h"
#include "Events/EDCreateObject.h"
#include <vector>

class PhysicsComponent;
class RenderComponent;
class EDCreateObject;

class CelestialObjectSystem : public ISystem
{
public:
	~CelestialObjectSystem();

	void Destroy()
	{
		Clear();
	}

	bool Init();
	void Update(float dt);
	void Clear();
	void CreateObject(IEventData* data);

	static std::string GetName()
	{
		return SYSTEM_NAME;
	};

private:
	std::vector<ICelestialObject*> m_objects;

	static const std::string SYSTEM_NAME;
};

#endif