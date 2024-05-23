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

#include "RenderToTexture.h"
#include "Shadowmap.h"

#include "../Utilities/HelperFunctions.h"
#include "../Events/EventSystem.h"
#include "../Events/EDCreateRenderComp.h"
#include "../Input/InputSystem.h"
//
//static const GLfloat g_quad_vertex_buffer_data[] =
//{
//	-1.0f, -1.0f, 0.0f,
//	1.0f, -1.0f, 0.0f,
//	-1.0f, 1.0f, 0.0f,
//	-1.0f, 1.0f, 0.0f,
//	1.0f, -1.0f, 0.0f,
//	1.0f, 1.0f, 0.0f,
//};


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

	//Render to texture program
	IRendererType* newRenderer = new RenderToTexture();
	shaderList.clear();
	shaderList.push_back(CreateShader(GL_VERTEX_SHADER, ReadFileToString("Vertex_Passthrough.glsl")));
	shaderList.push_back(CreateShader(GL_FRAGMENT_SHADER, ReadFileToString("RenderTexture.glsl")));
	GLuint rToTexProgram = CreateProgram(shaderList);
	newRenderer->Init(m_program, rToTexProgram);
	m_renderers[newRenderer->GetType()] = newRenderer;

	newRenderer = new Shadowmap();
	shaderList.clear();
	shaderList.push_back(CreateShader(GL_VERTEX_SHADER, ReadFileToString("Shadowmap_Vertex.glsl")));
	shaderList.push_back(CreateShader(GL_FRAGMENT_SHADER, ReadFileToString("Shadowmap_Fragment.glsl")));
	GLuint shadowProgram = CreateProgram(shaderList);
	newRenderer->Init(shadowProgram, rToTexProgram);
	m_renderers[newRenderer->GetType()] = newRenderer;

	//shaderList.clear();
	//shaderList.push_back(CreateShader(GL_VERTEX_SHADER, ReadFileToString("Vertex_Passthrough.glsl")));
	//shaderList.push_back(CreateShader(GL_FRAGMENT_SHADER, ReadFileToString("RenderTexture.glsl")));
	//m_quadProgID = CreateProgram(shaderList);

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

	//TODO: Write what these od for later
	
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

	for (auto it = m_renderers.begin(); it != m_renderers.end(); ++it)
	{
		delete it->second;
	}

	m_renderers.clear();
}

void RenderingSystem::ClearRender()
{
	m_physDebugDrawer.Clear();
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

void RenderingSystem::PreDraw(int drawOptions)
{
	RendererIterator it;
	m_projection = glm::perspective(45.0f, 4.0f / 3.0f, 0.1f, 200.0f);
	m_view = glm::lookAt(
		m_position,
		m_position + m_direction,
		m_up
		);
	m_MVP = m_projection * m_view *glm::mat4(1.f);
	GLuint MVPMatID = glGetUniformLocation(m_program, "MVP");
	GLuint err;
	if ((drawOptions &  Orbitals::DrawingType::TEXTURE) == Orbitals::DrawingType::TEXTURE)
	{
		it = m_renderers.find(Orbitals::DrawingType::TEXTURE);
		if (it == m_renderers.end())
		{
			// Do something
		}
		else
		{
			it->second->PreDraw();
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			GLuint VMatID = glGetUniformLocation(m_program, "V");
			glUniformMatrix4fv(VMatID, 1, GL_FALSE, &m_view[0][0]);

			GLuint lightPosID = glGetUniformLocation(m_program, "LightPosition_worldspace");
			glUniform3f(lightPosID, m_position.x, m_position.y, m_position.z);

			if ((drawOptions &  Orbitals::DrawingType::COMPONENTS) == Orbitals::DrawingType::COMPONENTS)
			{
				for (unsigned int i = 0; i < m_renderComponents.size(); ++i)
					m_renderComponents[i].Draw(m_view, m_projection, m_position);
			}
			if ((drawOptions &  Orbitals::DrawingType::PHYSICS) == Orbitals::DrawingType::PHYSICS)
				m_physDebugDrawer.Draw(m_MVP);
			while ((err = glGetError()) != GL_NO_ERROR)
			{
				std::cerr << "OpenGL error: " << err << std::endl;
			}
		}

	}

	if ((drawOptions & Orbitals::DrawingType::SHADOW_MAP) == Orbitals::DrawingType::SHADOW_MAP)
	{
		it = m_renderers.find(Orbitals::DrawingType::SHADOW_MAP);
		if (it == m_renderers.end())
		{
			// Do something
		}
		else
		{
			GLuint prog = it->second->PreDraw(glm::mat4(1.0f), glm::mat4(0.f), glm::vec3(m_spotPos), glm::vec3(m_spotDir));//m_directionLightDir);
			//GLuint prog = it->second->PreDraw(glm::mat4(1.0f), glm::mat4(0.f), glm::vec3(0.f), glm::vec3(m_directionLightDir));//m_directionLightDir);
			glm::mat4 depthVP = it->second->GetMVP();
			
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			if ((drawOptions &  Orbitals::DrawingType::COMPONENTS) == Orbitals::DrawingType::COMPONENTS)
			{
				glCullFace(GL_FRONT);
				for (unsigned int i = 0; i < m_renderComponents.size(); ++i)
				{
					m_renderComponents[i].Draw(depthVP, glm::mat4(1.f), m_position, prog);
				}
					
			}

			while ((err = glGetError()) != GL_NO_ERROR)
			{
				std::cerr << "OpenGL error: " << err << std::endl;
			}
		}

	}
}

void RenderingSystem::Draw(int drawOptions)
{
	GLenum err;
	RendererIterator it;

	//glUniformMatrix4fv(MVPMatID, 1, GL_FALSE, &m_MVP[0][0]);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glm::mat4 depthMVP(1.f);
	GLuint depthTex = 0;
	if ((drawOptions & Orbitals::DrawingType::SHADOW_MAP) == Orbitals::DrawingType::SHADOW_MAP)
	{
		it = m_renderers.find(Orbitals::DrawingType::SHADOW_MAP);
		if (it == m_renderers.end())
		{
			// Do something
		}
		else
		{
			//it->second->Draw();
			depthMVP = it->second->GetMVP();
			depthTex = it->second->GetTexutreID();
		}

	}

	glViewport(0, 0, 1024, 640);

	glUseProgram(m_program);
	glCullFace(GL_BACK);
	GLuint VMatID = glGetUniformLocation(m_program, "V");
	glUniformMatrix4fv(VMatID, 1, GL_FALSE, &m_view[0][0]);

	/*GLuint lightPosID = glGetUniformLocation(m_program, "LightPosition_worldspace");
	glUniform3f(lightPosID, m_directionLightDir.x, m_directionLightDir.y, m_directionLightDir.z);*/
	GLuint lightPosID = glGetUniformLocation(m_program, "lightPos");
	//glUniform4f(lightPosID, m_directionLightDir.x, m_directionLightDir.y, m_directionLightDir.z, m_directionLightDir.w);
	glUniform4f(lightPosID, m_spotPos.x, m_spotPos.y, m_spotPos.z, m_spotPos.w);

	GLuint viewPosID = glGetUniformLocation(m_program, "viewPos");
	glUniform3f(viewPosID, m_position.x, m_position.y, m_position.z);

	glm::mat4 biasMatrix(
		0.5, 0.0, 0.0, 0.0,
		0.0, 0.5, 0.0, 0.0,
		0.0, 0.0, 0.5, 0.0,
		0.5, 0.5, 0.5, 1.0
		);
	glm::mat4 depthBiasMVP = biasMatrix * depthMVP;
	GLuint DepthBiasID = glGetUniformLocation(m_program, "DepthMVP");
	glUniformMatrix4fv(DepthBiasID, 1, GL_FALSE, &depthBiasMVP[0][0]);

	//Bind shadowmap texuture
	if (depthTex != 0)
	{
		GLuint ShadowMapID = glGetUniformLocation(m_program, "shadowMap");
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, depthTex);
		glUniform1i(ShadowMapID, 0);
	}

	if ((drawOptions & Orbitals::DrawingType::COMPONENTS) == Orbitals::DrawingType::COMPONENTS)
	{
		for (unsigned int i = 0; i < m_renderComponents.size(); ++i)
			m_renderComponents[i].Draw(m_view, m_projection, m_position);
	}
	if ((drawOptions & Orbitals::DrawingType::PHYSICS) == Orbitals::DrawingType::PHYSICS)
		m_physDebugDrawer.Draw(m_MVP);

	while ((err = glGetError()) != GL_NO_ERROR)
	{
		std::cerr << "OpenGL error: " << err << std::endl;
	}
	return;
	if ((drawOptions & Orbitals::DrawingType::TEXTURE) == Orbitals::DrawingType::TEXTURE)
	{
		it = m_renderers.find(Orbitals::DrawingType::TEXTURE);
		if (it == m_renderers.end())
		{
			// Do something
		}
		else
		{
			it->second->Draw();
		}

	}

	if ((drawOptions & Orbitals::DrawingType::SHADOW_MAP) == Orbitals::DrawingType::SHADOW_MAP)
	{
		it = m_renderers.find(Orbitals::DrawingType::SHADOW_MAP);
		if (it == m_renderers.end())
		{
			// Do something
		}
		else
		{
			it->second->Draw();
			depthMVP = it->second->GetMVP();
			depthTex = it->second->GetTexutreID();
		}

	}

}

void RenderingSystem::CreateRenderComponent(IEventData* eventData)
{
	EDCreateRenderComp* renderCompED = dynamic_cast<EDCreateRenderComp*>(eventData);
	if (!renderCompED)
		return;

	RenderComponent newComponent((unsigned int)m_renderComponents.size());
	newComponent.SetVertices(renderCompED->GetData()->vertices);
	newComponent.SetNormals(renderCompED->GetData()->normals);
	newComponent.SetIndicies(renderCompED->GetData()->indicies);
	newComponent.SetColor(renderCompED->GetData()->color);
				
	newComponent.SetProgram(m_program);
	newComponent.SetDrawPrimitive(renderCompED->GetData()->drawType);
	

	m_renderComponents.push_back(newComponent);
	m_renderComponents[m_renderComponents.size() - 1].SetOwner(renderCompED->GetData()->owner);
	eventData->SetDelete(true);
}


const std::string RenderingSystem::SYSTEM_NAME = "RenderingSystem";