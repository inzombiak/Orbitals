#pragma once
#include <vector>
#include <string>

#include "HelperFunctions.h"
#include "RenderComponent.h"
#include "BTDebugDrawer.h"
#include "GameDefs.h"

class RenderComponent;
class IEventData;
class RenderManager
{
public:
	RenderManager();
	~RenderManager();
	//Initalize render manager
	void Init();
	//Clear existing render components
	void Clear();
	void ClearRender();
	void Step();
	RenderComponent* CreateRay(glm::vec3 start, glm::vec3 end);
	//Draw screen
	void Draw();
	btIDebugDraw* GetDebugDrawer()
	{
		if (!m_bulletDebugDrawer)
			return NULL;
		
		return m_bulletDebugDrawer;
	}

	glm::mat4 GetViewMat4() const
	{
		return glm::lookAt(
			m_position,
			m_position + m_direction,
			m_up
			);
	}

	//Create a render component
	void CreateRenderComponent(IEventData* eventData);


	void UpdateCameraRotation(float xRotate, float yRotate);
	void UpdateCameraPosition(std::map<GameDefs::GameKey, bool> keys);

private:
	BTDebugDrawer* m_bulletDebugDrawer;
	GLuint m_program;
	std::vector<RenderComponent*> m_renderComponents;
	//Used for camera speed and positioning
	glm::vec3 m_position = glm::vec3(0, 3, 10); 
	glm::vec3 m_right = glm::vec3(1, 1, 0);
	glm::vec3 m_up = glm::vec3(0, 1, 0);
	glm::vec3 m_direction = glm::vec3(-4, -3, 3);
	float m_speed = .8f;
	float m_mouseSpeed = 0.005f;
};