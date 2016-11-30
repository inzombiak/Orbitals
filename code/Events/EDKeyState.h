#ifndef ED_KEY_STATE_H
#define ED_KEY_STATE_H

#pragma once
#include "IEventData.h"
#include "../Utilities/OrbitalsDefs.h"

class EDKeyState :
	public IEventData
{
public:

	EDKeyState(bool isDown, Orbitals::InputKey key)
	{
		m_isDown = isDown;
		m_key = key;
	}

	bool IsDown()
	{
		return m_isDown;
	}

	Orbitals::InputKey GetKey()
	{
		return m_key;
	}

private:
	bool m_isDown;
	Orbitals::InputKey m_key;
};

#endif