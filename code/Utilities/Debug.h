#ifndef DEBUG_H
#define DEBUG_H

#include "../Metaprogramming/Common/Singleton.h"

#include <ctime>
#include <string>

class Debug
{
public:
	void StartTimer(std::string);
	void EndTimer(std::string);
	void PrintMessage(std::string);

	void DisplayFPS(float dt);

private:
	int m_frames = 0;
	float m_time = 0;
	std::clock_t m_clock = std::clock_t();
	bool m_clockRunning = false;
};

typedef SingletonHolder<Debug, CreationPolicies::CreateWithNew, LifetimePolicies::DefaultLifetime> ORB_DBG;

#endif