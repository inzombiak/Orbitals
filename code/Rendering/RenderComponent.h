#ifndef RENDER_COMPONENT_H
#define RENDER_COMPONENT_H

#include "../Objects/IObjectComponent.h"

#define GLEW_STATIC
#include <GL\glew.h>

class RenderComponent :
	public IObjectComponent
{
public:
	RenderComponent(unsigned int id)
	{
		m_id = id;
	};
	//Draws buffer
	void Draw(glm::mat4 view, glm::mat4 proj, glm::vec3 lightPos);
	//Update vertex buffer
	void SetVertices(const std::vector<glm::vec3>& vertices);
	//Update color buffer
	void SetColor(std::vector<glm::vec3> color);
	//Update normals
	void SetNormals(const std::vector<glm::vec3>& normals);
	//Update indicied
	void SetIndicies(const std::vector<unsigned short>& indicies);
	//Set draw type
	void SetDrawPrimitive(GLuint type);
	//Set OpenGL program
	void SetProgram(GLuint program);
	//Set model matrix
	void SetModelMatrix(glm::mat4 matrix)
	{
		m_model = matrix;
	}
	void Update(float dt) override;
	const char* GetName() override
	{
		return COMPONENT_NAME;
	}
	ObjComponentID GetComponentID() override
	{
		return COMPONENT_ID;
	}
private:
	const static char* COMPONENT_NAME;
	const static ObjComponentID COMPONENT_ID;

	//Number of vertices to be drawn
	int m_numVertices;
	int m_numIndicies;
	glm::mat4 m_model;
	//Program ID
	GLuint m_program;
	//Buffer IDs
	GLuint m_vertexBufferObject = 0;
	GLuint m_colorBufferObject = 0;
	GLuint m_normalBufferObject = 0;
	GLuint m_indexBufferObject = 0;
	//Draw type
	GLuint m_drawPrimitive;
};

#endif