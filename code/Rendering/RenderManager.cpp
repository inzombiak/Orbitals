#define GLEW_STATIC
#include <gl\glew.h>

#include <glm\glm.hpp>
#include <glm\gtc\matrix_transform.hpp>
#include <qopenglcontext.h>
#include <qkeyevent>

#include "RenderManager.h"
#include "BTDebugDrawer.h"
#include "RenderComponent.h"
#include "HelperFunctions.h"
#include "QtOpenGLWidget.h"
#include "EventManager.h"
#include "EDCreateRenderComp.h"

RenderManager::RenderManager()
{
	m_bulletDebugDrawer = 0;
}


RenderManager::~RenderManager()
{
	Clear(); 
}

void RenderManager::Init()
{
	if (!m_bulletDebugDrawer)
		m_bulletDebugDrawer = new BTDebugDrawer();
	glewInit();
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	EventManager::GetInstance()->AddEventListener(EventDefs::CREATE_RENDER_COMPONENT, std::bind(&RenderManager::CreateRenderComponent, this, std::placeholders::_1));

	//Create shaders and add to vector
	std::vector<GLuint> shaderList;
	shaderList.push_back(CreateShader(GL_VERTEX_SHADER, ReadFileToString("VertexShader.glsl")));
	shaderList.push_back(CreateShader(GL_FRAGMENT_SHADER, ReadFileToString("FragmentShader.glsl")));

	//Create program
	m_program = CreateProgram(shaderList);
	m_bulletDebugDrawer->SetProgram(m_program);
}
void RenderManager::Clear()
{
	if (m_bulletDebugDrawer)
		delete m_bulletDebugDrawer;

	for (unsigned int i = 0; i < m_renderComponents.size(); ++i)
	{
		delete m_renderComponents[i];
		m_renderComponents[i] = 0;
	}

	m_renderComponents.clear();
}

void RenderManager::ClearRender()
{
	if (m_bulletDebugDrawer)
		m_bulletDebugDrawer->Clear();
}

void RenderManager::Step()
{
	for (unsigned int i = 0; i < m_renderComponents.size(); ++i)
		m_renderComponents[i]->Update();
}

void RenderManager::UpdateCameraPosition(std::map<GameDefs::GameKey, bool> keys)
{
	if(keys[GameDefs::GameKey::W])
		m_position += m_direction  * m_speed;
	if (keys[GameDefs::GameKey::S])
		m_position -= m_direction  * m_speed;
	if (keys[GameDefs::GameKey::A])
		m_position -= m_right  * m_speed;
	if (keys[GameDefs::GameKey::D])
		m_position += m_right  * m_speed;
}

void RenderManager::UpdateCameraRotation(float xRotate, float yRotate)
{

	m_direction = glm::vec3(
		cos(yRotate) * sin(xRotate),
		sin(yRotate),
		cos(yRotate) * cos(xRotate)
		);

	m_right = glm::vec3(
		sin(xRotate - 3.14f / 2.0f),
		0,
		cos(xRotate - 3.14f / 2.0f)
		);
	m_up = glm::cross(m_right, m_direction);

}

void RenderManager::Draw()
{
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

	//for (unsigned int i = 0; i < m_renderComponents.size(); ++i)
		//m_renderComponents[i]->Draw(vp);

	m_bulletDebugDrawer->Draw(vp);

}

void RenderManager::CreateRenderComponent(IEventData* eventData)
{
	//Create a new render component and return it
	RenderComponent* newComponent = new RenderComponent(m_renderComponents.size());

	EDCreateRenderComp* renderCompED = dynamic_cast<EDCreateRenderComp*>(eventData);
	if (!renderCompED)
		return;

	newComponent->SetVertices(renderCompED->GetData()->vertices);
	newComponent->SetColor(renderCompED->GetData()->color);
	newComponent->SetProgram(m_program);
	newComponent->SetDrawPrimitive(renderCompED->GetData()->drawType);
	newComponent->SetOwner(renderCompED->GetData()->owner);

	m_renderComponents.push_back(newComponent);

	eventData->SetDelete(true);
}


RenderComponent* RenderManager::CreateRay(glm::vec3 start, glm::vec3 end)
{
	RenderComponent* newComponent = new RenderComponent(m_renderComponents.size());
	std::vector<glm::vec3> temp;
	newComponent->SetDrawPrimitive(GL_LINES);

	temp.push_back(start);
	temp.push_back(end);
	newComponent->SetVertices(temp);

	temp.clear();
	temp.push_back(glm::vec3(1.f, 0.f, 0.f));
	newComponent->SetColor(temp);

	m_renderComponents.push_back(newComponent);
	return newComponent;
}