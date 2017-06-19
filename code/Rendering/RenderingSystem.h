#ifndef RENDERING_SYSTEM_H
#define RENDERING_SYSTEM_H

#include <map>

#include "../ISystem.h"
#include "..\Utilities\HelperFunctions.h"

#include "IRendererType.h"
#include "RenderComponent.h"
#include "PhysDebugDrawer.h"

class IEventData;
class RenderingSystem : public ISystem
{
public:
	~RenderingSystem();
	//Initalize render manager
	bool Init() override;
	void Destroy() override
	{
		Clear();
	}
	void Update(float dt) override;

	//Clear existing render components
	void Clear();
	void ClearRender();
	
	PhysDebugDrawer* GetPhysDebugDrawer()
	{
		return &m_physDebugDrawer;
	}

	//Draw screen
	void Draw(int drawOptions);
	void PreDraw(int drawOptions);

	SystemPriority GetPriority()
	{
		return Orbitals::SystemPriority::SRendering;
	}


	static std::string GetName()
	{
		return SYSTEM_NAME;
	};

	//Create a render component
	void CreateRenderComponent(IEventData* eventData);

private:

	void DrawComponent(int index, glm::mat4 view, glm::mat4 proj, glm::vec3 lightPos);

	void UpdateCameraRotation();
	void UpdateCameraPosition();

	GLuint m_program;


	//TEST
	//GLuint m_quad_VertexArrayID;


	PhysDebugDrawer m_physDebugDrawer;

	std::vector<RenderComponent> m_renderComponents;
	std::map<Orbitals::DrawingType, IRendererType*> m_renderers;
	typedef std::map<Orbitals::DrawingType, IRendererType*>::iterator RendererIterator;
	//Used for camera speed and positioning

	//Test lights, need to make rendering syste, more flexible
	glm::vec3 m_directionLightDir = glm::vec3(-2.0f, 1.0f, -1.0f);;
	glm::vec3 m_spotPos = glm::vec3(-8, 6, -4);

	glm::vec3 m_position = glm::vec3(-15, 16, 10);
	glm::vec3 m_right = glm::vec3(0, 0, 1);
	glm::vec3 m_up = glm::vec3(0, 1, 0);
	glm::vec3 m_direction = glm::vec3(1, 0, 0);
	glm::mat4 m_view;
	glm::mat4 m_projection;
	glm::mat4 m_MVP;
	float m_speed = .04f;
	float m_mouseSpeed = 0.005f;
	bool m_trackingMouse = false;
	float m_xRotate = 2.515f, m_yRotate = -0.475f;
	glm::vec2 m_prevMouse;
	static const std::string SYSTEM_NAME;
};

#endif
