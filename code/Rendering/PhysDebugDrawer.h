#ifndef PHYS_DEBUG_DRAWER
#define PHYS_DEBUG_DRAWER

#define GLEW_STATIC
#include <GL\glew.h>
#include <glm/glm.hpp>

#include <vector>

class PhysDebugDrawer
{
public:
	void SetProgram(GLuint program);
	void Clear();

	void Draw(glm::mat4& mvp);
	void DrawLine(const glm::vec3& from, const glm::vec3& to, const glm::vec3& color);
	void DrawPoint(const glm::vec3& p, float size, const glm::vec3& color);
	void DrawSphereShape();
	void DrawBoxShape();
	void DrawAABB(const glm::vec3& min, const glm::vec3& max, const glm::vec3& color);
	
	void SetDebugMode(int debugMode);
	int GetDebugMode() const;

private:
	std::vector<glm::vec3> m_vertices;
	std::vector<glm::vec3> m_colors;

	//TODO ADD FLAGS
	int m_debugMode = 0;

	GLuint m_program = 0;

};

#endif

