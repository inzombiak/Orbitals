#ifndef I_SYSTEM_H
#define I_SYSTEM_H

#include "Utilities\OrbitalsDefs.h"

class ISystem
{
public:
	ISystem() {};
	virtual ~ISystem() {};

	virtual void Destroy() = 0;
	virtual bool Init() = 0;
	virtual void Update(float dt) = 0;

private:
	SystemID m_id;
};

#endif