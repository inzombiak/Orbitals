#include "InputSystem.h"

void InputSystem::SetKeyDown(Input::InputKey key, bool value)
{
	m_keyMap[key] = value;
}

bool InputSystem::GetKeyDown(Input::InputKey key) const
{
	auto it = m_keyMap.find(key);

	if (it == m_keyMap.end())
		return false;
	else
		return it->second;
	

}

void InputSystem::SetMousePosition(glm::vec2 position)
{
	m_mouseStatus.prevPosition = m_mouseStatus.position;
	m_mouseStatus.position = position;
}

void InputSystem::SetMouseLeftButton(bool value)
{
	m_mouseStatus.leftBtnDown = value;
}

void InputSystem::SetMouseRightButton(bool value)
{
	m_mouseStatus.rightBtnDown = value;
}

void InputSystem::SetMouseWheelButton(bool value)
{
	m_mouseStatus.mouseWheelBtnDown = value;
}

void InputSystem::SetMouseWheelRotation(float value)
{
	m_mouseStatus.mouseWheelRotation = value;
}

Input::MouseStatus InputSystem::GetMouseStatus() const
{
	return m_mouseStatus;
}


const std::string InputSystem::SYSTEM_NAME = "InputSystem";
InputSystem* InputSystem::m_instance = 0;