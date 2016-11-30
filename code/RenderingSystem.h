#ifndef RENDERING_SYSTEM_H
#define RENDERING_SYSTEM_H

#include "ISystem.h"
#include "Utilities\HelperFunctions.h"

#include <unordered_map>

class RenderComponent;
class IEventData;
class RenderingSystem : public ISystem
{
public:
	~RenderingSystem();
	//Initalize render manager
	bool Init();

	void Destroy()
	{
		Clear();
	}

	//Clear existing render components
	void Clear();
	void ClearRender();
	RenderComponent* CreateRay(glm::vec3 start, glm::vec3 end);

	//Draw screen
	void Draw();

	void Update(float dt);
	static std::string GetName()
	{
		return SYSTEM_NAME;
	};

	//Create a render component
	void CreateRenderComponent(IEventData* eventData);

private:

	void UpdateCameraRotation();
	void UpdateCameraPosition();

	GLuint m_program;

	std::vector<RenderComponent*> m_renderComponents;

	//Used for camera speed and positioning
	glm::vec3 m_position = glm::vec3(-6, 0, -4);
	glm::vec3 m_right = glm::vec3(0, 0, 1);
	glm::vec3 m_up = glm::vec3(0, 1, 0);
	glm::vec3 m_direction = glm::vec3(1, 0, 0);
	glm::mat4 m_projection;
	float m_speed = .008f;
	float m_mouseSpeed = 0.005f;
	bool m_trackingMouse = false;
	float m_xRotate = 0, m_yRotate = 0;
	glm::vec2 m_prevMouse;
	static const std::string SYSTEM_NAME;
};

#endif
