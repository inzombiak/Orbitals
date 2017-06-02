#include "Debug.h"

#include <iostream>

void Debug::StartTimer(std::string displayText)
{
	if (!m_clockRunning)
	{
		std::cout << displayText << std::endl;
		m_clock = std::clock();
		m_clockRunning = true;

		return;
	}

	std::cout << "CLOCK ALREADY RUNNING" << std::endl;
}

void Debug::EndTimer(std::string displayText)
{
	if (m_clockRunning)
	{
		double duration = (std::clock() - m_clock) / (double)CLOCKS_PER_SEC;
		std::cout << displayText << duration << std::endl;
		m_clockRunning = false;

		return;
	}

	std::cout << "CLOCK NOT RUNNING" << std::endl;
}

void Debug::PrintMessage(std::string message)
{
	std::cout << message << std::endl;
}

void Debug::DisplayFPS(float dt)
{
	m_frames++;
	m_time += dt;
	if (m_time >= 1)
	{
		std::cout << "FPS: " << m_frames - 1 << std::endl;
		m_frames = 0;
		m_time = 0;
	}

}