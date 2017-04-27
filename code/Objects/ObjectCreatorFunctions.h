#ifndef OBJECT_CREATOR_FUNCTIONS
#define OBJECT_CREATOR_FUNCTIONS

#include "../Metaprogramming/Factories.h"
#include "../Metaprogramming/Common/Singleton.h"
#include "../Events/EDCreateObject.h"

class EDCreateObject;
namespace ObjectCreators
{
	//TODO: Is this redundant? Merge with PhysicsDefs.h?

	enum DefaultShapeType
	{
		Sphere = 1,
		Box = 2,
	};

	class IDefaultShapeData
	{
	public:
		virtual ~IDefaultShapeData(){}
		virtual DefaultShapeType GetType() = 0;
		glm::vec3 position;
		glm::vec3 rotation;
		glm::vec3 scale;
		glm::vec3 color;
	};

	class SphereShapeData : public IDefaultShapeData
	{
	public:
		DefaultShapeType GetType()
		{
			return Sphere;
		}
		float radius;

	};

	class BoxShapeData : public IDefaultShapeData
	{
	public:
		DefaultShapeType GetType()
		{
			return Box;
		}
		glm::vec3 extents;

	};

	EDCreateObject* CreateSphereEventData(IDefaultShapeData* data);
	EDCreateObject* CreateBoxEventData(IDefaultShapeData* data);

	typedef SingletonHolder<Factory<EDCreateObject, DefaultShapeType, TYPELIST_1(ObjectCreators::IDefaultShapeData*)>, CreationPolicies::CreateWithNew, LifetimePolicies::DefaultLifetime> DefaultObjectFactory;
	
}

#endif