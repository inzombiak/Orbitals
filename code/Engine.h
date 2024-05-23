#ifndef ENGINE_H
#define ENGINE_H

#include "Utilities/OrbitalsDefs.h"
#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <map>

class ISystem;
class Engine
{
	enum EngineState
	{
		Initializing,
		Running,
		Closing,
		Reseting,
	};

public:
	Engine();
	~Engine();
	void Init();
	void Draw();
	void Step(float timePassed);

	void SetKey(Input::InputKey key, bool value);
	void SetMousePosition(int x, int y);
	void SetMouseWheelRotation(float rot);
	void SetLeftBtn(bool value);
	void SetRightBtn(bool value);
	void SetMiddleBtn(bool value);

	void SetScreenSize(int screenWidth, int screenHeight)
	{
		m_screenWidth = screenWidth;
		m_screenHeight = screenHeight;
	}

private:

	template<class System>
	bool FindSystem(Orbitals::SystemPriority sysId, std::shared_ptr<System>& result)//ConvertToStrongPtr and CastComponentToDerived in one function!
	{
		auto it = m_systemMap.find(sysId);
		if (it == m_systemMap.end())
			return false;

		std::shared_ptr<System> ptr;
		if (CheckConvertAndCastPtr<ISystem, System>(it->second, ptr))
			result = ptr;
		else
			return false;

		return true;
	}

	void Test();
	void InitSim();
	void Reset();
	void UpdateCameraRotation(float xRotate, float yRotate);
	void ClearRender();
	//WeakSystemPtr GetSystemWithByID
	Input::MouseStatus m_mouseStatus = Input::MouseStatus();
	int m_screenWidth = 0, m_screenHeight = 0;
	int m_drawOptions = 0;
	const double DELTA_T = 1.f / 30.f;
	double m_accumulator = 0;
	int m_numberOfSubSteps = 10;
	EngineState m_engineState = EngineState::Initializing;
	std::map<Orbitals::SystemPriority, StrongSystemPtr> m_systemMap;
};

#endif