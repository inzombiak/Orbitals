#include "PhysDebugDrawer.h"

void PhysDebugDrawer::SetProgram(GLuint program)
{
	m_program = program;
}
void PhysDebugDrawer::Clear()
{
	m_vertices.clear();
	m_colors.clear();
}

void PhysDebugDrawer::Draw(glm::mat4& mvp)
{
	if (m_vertices.size() == 0)
		return;
	//Set program
	glUseProgram(m_program);
	mvp = mvp * glm::mat4(1.0f);

	GLuint vertexBufferObject = 0;

	glGenBuffers(1, &vertexBufferObject);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);
	glBufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(glm::vec3), &m_vertices[0], GL_STATIC_DRAW);

	GLuint colorBufferObject = 0;
	glGenBuffers(1, &colorBufferObject);
	glBindBuffer(GL_ARRAY_BUFFER, colorBufferObject);
	glBufferData(GL_ARRAY_BUFFER, m_colors.size() * sizeof(glm::vec3), &m_colors[0], GL_STATIC_DRAW);

	//Enable attributes for use
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);
	glVertexAttribPointer(
		0,
		3,
		GL_FLOAT,
		GL_FALSE,
		0,
		(void*)0
		);

	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, colorBufferObject);
	glVertexAttribPointer(
		1,
		3,
		GL_FLOAT,
		GL_FALSE,
		0,
		(void*)0
		);

	//Set draw mode
	//glPolygonMode(GL_FRONT, GL_LINE);

	//Pass in MVP
	GLuint MVPMatID = glGetUniformLocation(m_program, "MVP");
	glUniformMatrix4fv(MVPMatID, 1, GL_FALSE, &mvp[0][0]);

	//Draw
	glDrawArrays(GL_LINES, 0, (GLsizei)m_vertices.size());
}


void PhysDebugDrawer::DrawLine(const glm::vec3& from, const glm::vec3& to, const glm::vec3& color)
{
	m_vertices.push_back(from);
	m_vertices.push_back(to);

	m_colors.push_back(color);
	m_colors.push_back(color);
}
void PhysDebugDrawer::DrawPoint(const glm::vec3& p, float size, const glm::vec3& color)
{
	m_vertices.push_back(p - glm::vec3(1, 0, 0) * size);
	m_vertices.push_back(p + glm::vec3(1, 0, 0) * size);
	m_vertices.push_back(p - glm::vec3(0, 1, 0) * size);
	m_vertices.push_back(p + glm::vec3(0, 1, 0) * size);
	m_vertices.push_back(p - glm::vec3(0, 0, 1) * size);
	m_vertices.push_back(p + glm::vec3(0, 0, 1) * size);

	m_colors.push_back(color);
	m_colors.push_back(color);
	m_colors.push_back(color);
	m_colors.push_back(color);
	m_colors.push_back(color);
	m_colors.push_back(color);

}
void PhysDebugDrawer::DrawSphereShape()
{
	//TODO Implement
}
void PhysDebugDrawer::DrawBoxShape()
{
	//TODO Implement
}
void PhysDebugDrawer::DrawAABB(const glm::vec3& min, const glm::vec3& max, const glm::vec3& color)
{
	glm::vec3 center = (min + max) / 2.f;
	glm::vec3 halfExtents = (max - min) / 2.f;

	glm::vec3 currentVertex(-1, -1, -1);
	glm::vec3 from, to;
	static const glm::vec3 OFFSET(-0.01f, -0.01f, -0.01f);
	for (unsigned int i = 0; i < 4; ++i)
	{ 
		from = (currentVertex * (halfExtents - OFFSET)) + center;
		for (unsigned int j = 0; j < 3; ++j)
		{
			to = from;
			to[j] -= 2 * (currentVertex[j] * (halfExtents[j] - OFFSET[j]));
			
			DrawLine(from, to, color);
		}

		currentVertex[(i % 2) + 1] *= -1;
		currentVertex[((i + 1) % 2 )* 2] *= -1;
	}

}

void PhysDebugDrawer::SetDebugMode(int debugMode)
{
	m_debugMode = debugMode;
};
int PhysDebugDrawer::GetDebugMode() const
{
	return m_debugMode;
};