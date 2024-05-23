#include "RenderComponent.h"
#include "../Objects/ICelestialObject.h"

void RenderComponent::Draw(glm::mat4 view, glm::mat4 proj, glm::vec3 lightPos, GLuint program, bool assignUniforms)
{
	//Set program
	//glUseProgram(m_program);
	if (program == -1)
		program = m_program;
	//Enable attributes for use
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);
	glVertexAttribPointer(
		0,
		3,
		GL_FLOAT,
		GL_FALSE,
		0,
		(void*)0
		);

	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, m_colorBufferObject);
	glVertexAttribPointer(
		1,
		3,
		GL_FLOAT,
		GL_FALSE,
		0,
		(void*)0
		);

	glEnableVertexAttribArray(2);
	glBindBuffer(GL_ARRAY_BUFFER, m_normalBufferObject);
	glVertexAttribPointer(
		2,                                // attribute
		3,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
		);
		

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBufferObject);

	//Set draw mode
	//if (m_drawPrimitive == GL_POINTS)
	//	glPolygonMode(GL_FRONT, GL_POINT);
	//else
	//	glPolygonMode(GL_FRONT, GL_FILL);

	m_model = m_owner->GetOpenGLMatrix();

	glm::mat4 mvp = proj * view * m_model;
	//Pass in MVP
	if (assignUniforms)
	{
		GLuint MVPMatID = glGetUniformLocation(program, "MVP");
		glUniformMatrix4fv(MVPMatID, 1, GL_FALSE, &mvp[0][0]);

	}

	GLuint MMatID = glGetUniformLocation(program, "M");
	glUniformMatrix4fv(MMatID, 1, GL_FALSE, &m_model[0][0]);
	glm::mat3 transInverseM = glm::transpose(glm::inverse(glm::mat3(m_model)));
	GLuint TIMMatID = glGetUniformLocation(program, "transInverseM");
	glUniformMatrix3fv(TIMMatID, 1, GL_FALSE, &transInverseM[0][0]);
	//Draw
	if (m_numIndicies >= 0)
	{
		glDrawElements(
			GL_TRIANGLES,      // mode
			m_numIndicies,    // count
			GL_UNSIGNED_SHORT,   // type
			(void*)0           // element array buffer offset
			);
	}
	else
		glDrawArrays(m_drawPrimitive, 0, m_numVertices);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);

}

void RenderComponent::Update(float dt)
{

}

void RenderComponent::SetProgram(GLuint program)
{
	m_program = program;
}

void RenderComponent::SetVertices(const std::vector<glm::vec3>& vertices)
{
	if (vertices.size() <= 0)
	{
		m_numVertices = 0;
		return;
	}

	//If no buffer exists, create one
	if (m_vertexBufferObject == 0)
		glGenBuffers(1, &m_vertexBufferObject);
	GLenum error = glGetError();
	//Set data
	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferObject);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);
	m_numVertices = (int)vertices.size();
}

void RenderComponent::SetColor(std::vector<glm::vec3> color)
{
	if (color.size() <= 0)
		return;
	//Same as above
	if (m_colorBufferObject == 0)
		glGenBuffers(1, &m_colorBufferObject);

	while (color.size() < m_numVertices)
		color.push_back(color[0]);

	GLenum error = glGetError();
	glBindBuffer(GL_ARRAY_BUFFER, m_colorBufferObject);
	glBufferData(GL_ARRAY_BUFFER, color.size() * sizeof(glm::vec3), &color[0], GL_STATIC_DRAW);
}


void RenderComponent::SetNormals(const std::vector<glm::vec3>& normals)
{
	if (normals.size() <= 0)
		return;

	if (m_normalBufferObject == 0)
		glGenBuffers(1, &m_normalBufferObject);

	glBindBuffer(GL_ARRAY_BUFFER, m_normalBufferObject);
	glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), &normals[0], GL_STATIC_DRAW);
}

void RenderComponent::SetIndicies(const std::vector<unsigned short>& indices)
{
	if (indices.size() <= 0)
		return;

	if (m_indexBufferObject == 0)
		glGenBuffers(1, &m_indexBufferObject);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBufferObject);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned short), &indices[0], GL_STATIC_DRAW);
	m_numIndicies = (int)indices.size();
}

void RenderComponent::SetDrawPrimitive(GLuint type)
{
	m_drawPrimitive = type;
}

const char*  RenderComponent::COMPONENT_NAME = "RENDER_COMPONENT";
const ObjComponentID RenderComponent::COMPONENT_ID = Orbitals::HashedString::hash_name(RenderComponent::COMPONENT_NAME);