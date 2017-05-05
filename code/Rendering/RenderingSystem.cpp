#include "RenderingSystem.h"

#define GLEW_STATIC
#include <gl\glew.h>
#include <gl\GL.h>
#include <fstream>
#include <iostream>

#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include "RenderingSystem.h"
#include "RenderComponent.h"
#include "../Utilities/HelperFunctions.h"
#include "../Events/EventSystem.h"
#include "../Events/EDCreateRenderComp.h"
#include "../Input/InputSystem.h"


RenderingSystem::~RenderingSystem()
{
	Clear();
}

bool RenderingSystem::Init()
{
	//glewInit();
	glEnable(GL_CULL_FACE);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glCullFace(GL_BACK);
	glFrontFace(GL_CCW);
	EventSystem::GetInstance()->AddEventListener(EventDefs::CREATE_RENDER_COMPONENT, std::bind(&RenderingSystem::CreateRenderComponent, this, std::placeholders::_1));

	std::vector<GLuint> shaderList;
	shaderList.push_back(CreateShader(GL_VERTEX_SHADER, ReadFileToString("VertexShader_Lighting.glsl")));
	shaderList.push_back(CreateShader(GL_FRAGMENT_SHADER, ReadFileToString("FragmentShader_Lighting.glsl")));
	m_program = CreateProgram(shaderList);

	//Debug drawing program
	shaderList.clear();
	shaderList.push_back(CreateShader(GL_VERTEX_SHADER, ReadFileToString("VertexShader.glsl")));
	shaderList.push_back(CreateShader(GL_FRAGMENT_SHADER, ReadFileToString("FragmentShader.glsl")));
	GLuint program = CreateProgram(shaderList);
	m_physDebugDrawer.SetProgram(program);

	m_direction = glm::vec3(
		cos(m_yRotate) * sin(m_xRotate),
		sin(m_yRotate),
		cos(m_yRotate) * cos(m_xRotate)
		);

	m_right = glm::vec3(
		sin(m_xRotate - 3.14f / 2.0f),
		0,
		cos(m_xRotate - 3.14f / 2.0f)
		);
	m_up = glm::cross(m_right, m_direction);


	return true;
}
void RenderingSystem::Clear()
{
	//for (unsigned int i = 0; i < m_renderComponents.size(); ++i)
	//{
	//	delete m_renderComponents[i];
	//	m_renderComponents[i] = 0;
	//}

	m_renderComponents.clear();
}

void RenderingSystem::ClearRender()
{
	
}

void RenderingSystem::Update(float dt)
{
	UpdateCameraPosition();
	UpdateCameraRotation();

	for (unsigned int i = 0; i < m_renderComponents.size(); ++i)
		m_renderComponents[i].Update(dt);
}

void RenderingSystem::UpdateCameraPosition()
{
	auto is = InputSystem::GetInstance();

	if (is->GetKeyDown(Input::InputKey::Forward))
		m_position += m_direction  * m_speed;
	if (is->GetKeyDown(Input::InputKey::Backwards))
		m_position -= m_direction  * m_speed;
	if (is->GetKeyDown(Input::InputKey::Left))
		m_position -= m_right  * m_speed;
	if (is->GetKeyDown(Input::InputKey::Right))
		m_position += m_right  * m_speed;
}

void RenderingSystem::UpdateCameraRotation()
{
	auto mouseStatus = InputSystem::GetInstance()->GetMouseStatus();
	if (!mouseStatus.leftBtnDown)
	{
		m_trackingMouse = false;
		return;
	}
	else if (!m_trackingMouse)
	{
		m_prevMouse = mouseStatus.position;
		m_trackingMouse = true;
		return;
	}
	if (glm::length(mouseStatus.position - m_prevMouse) <= 0.1)
		return;

	float xRotate = (mouseStatus.position.x -m_prevMouse.x) * m_mouseSpeed;
	float yRotate = (mouseStatus.position.y -m_prevMouse.y) * m_mouseSpeed;
	
	m_xRotate += xRotate;
	m_yRotate += yRotate;

	m_direction = glm::vec3(
		cos(m_yRotate) * sin(m_xRotate),
		sin(m_yRotate),
		cos(m_yRotate) * cos(m_xRotate)
		);

	m_right = glm::vec3(
		sin(m_xRotate - 3.14f / 2.0f),
		0,
		cos(m_xRotate - 3.14f / 2.0f)
		);
	m_up = glm::cross(m_right, m_direction);
	m_prevMouse = mouseStatus.position;
}

void RenderingSystem::Draw()
{
	std::cout << "DRAW START" << std::endl;
	glUseProgram(m_program);
	glClearColor(.7f, 0.7f, 1.0f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//MVP matrix, for 2D drawing this is all we need
	glm::mat4 vp(1.0f);

	glm::mat4 projection = glm::perspective(45.0f, 4.0f / 3.0f, 0.1f, 200.0f);
	glm::mat4 view = glm::lookAt(
		m_position,
		m_position + m_direction,
		m_up
		);
	vp = projection * view;

	auto MVP = projection * view * glm::mat4(1.f);
	GLuint MVPMatID = glGetUniformLocation(m_program, "MVP");
	glUniformMatrix4fv(MVPMatID, 1, GL_FALSE, &MVP[0][0]);
	std::cout << "DRAW MID" << std::endl;
	//glDisableVertexAttribArray(0);
	//glDisableVertexAttribArray(1);
	
	for (unsigned int i = 0; i < m_renderComponents.size(); ++i)
		m_renderComponents[i].Draw(view, projection, m_position);

	GLenum err;
	while ((err = glGetError()) != GL_NO_ERROR) 
	{
		std::cerr << "OpenGL error: " << err << std::endl;
	}

	m_physDebugDrawer.Draw(MVP);
	m_physDebugDrawer.Clear();

	std::cout << "DRAW END" << std::endl;
}

void RenderingSystem::CreateRenderComponent(IEventData* eventData)
{
	EDCreateRenderComp* renderCompED = dynamic_cast<EDCreateRenderComp*>(eventData);
	if (!renderCompED)
		return;

	RenderComponent newComponent(m_renderComponents.size());
	newComponent.SetVertices(renderCompED->GetData()->vertices);
	newComponent.SetNormals(renderCompED->GetData()->normals);
	newComponent.SetIndicies(renderCompED->GetData()->indicies);
	newComponent.SetColor(renderCompED->GetData()->color);
				
	newComponent.SetProgram(m_program);
	newComponent.SetDrawPrimitive(renderCompED->GetData()->drawType);
	newComponent.SetOwner(renderCompED->GetData()->owner);

	m_renderComponents.push_back(newComponent);

	eventData->SetDelete(true);
}


const std::string RenderingSystem::SYSTEM_NAME = "RenderingSystem";