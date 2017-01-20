#ifndef INPUT_SYSTEM_H
#define INPUT_SYSTEM_H

#include "X:\code\ISystem.h"
#include <unordered_map>

class InputSystem :
	public ISystem
{
public:
	bool Init()
	{
		m_mouseStatus.leftBtnDown = false;
		m_mouseStatus.rightBtnDown = false;
		m_mouseStatus.mouseWheelBtnDown = false;
		return true;
	};

	void Update(float dt) {};
	
	void SetKeyDown(Input::InputKey key, bool value);
	bool GetKeyDown(Input::InputKey key) const;

	void SetMousePosition(glm::vec2 position);
	void SetMouseLeftButton(bool value);
	void SetMouseRightButton(bool value);
	void SetMouseWheelButton(bool value);
	void SetMouseWheelRotation(float value);

	Input::MouseStatus GetMouseStatus() const;

	void Destroy()
	{
		//DestroyInstance();
	};

	SystemPriority GetPriority()
	{
		return Orbitals::SystemPriority::SInput;
	}

	static std::string GetName()
	{
		return SYSTEM_NAME;
	};

	static InputSystem* GetInstance()
	{
		if (!m_instance)
			m_instance = new InputSystem();

		return m_instance;
	}

	static void DestroyInstance()
	{
		if (m_instance)
			delete m_instance;

		m_instance = 0;
	};

private:

	InputSystem() {};
	static const std::string SYSTEM_NAME;
	static InputSystem* m_instance;

	std::unordered_map<Input::InputKey, bool> m_keyMap;
	Input::MouseStatus m_mouseStatus;
};

#endif
