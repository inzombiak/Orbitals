#ifndef SHADOWMAP_H
#define SHADOWMAP_H

#include "IRendererType.h"

class Shadowmap : public IRendererType
{

public:
	Shadowmap()
	{
		m_type = Orbitals::DrawingType::SHADOW_MAP;
	}

	void Init(GLuint preDrawProgram, GLuint drawProgram) override;
	GLuint Draw(const glm::mat4& proj = glm::mat4(1.f), const glm::mat4& view = glm::mat4(1.f), const glm::vec3& lightPos = glm::vec3(0), const glm::vec3& lightDir = glm::vec3(0)) override;
	GLuint PreDraw(const glm::mat4& proj = glm::mat4(1.f), const glm::mat4& view = glm::mat4(1.f), const glm::vec3& lightPos = glm::vec3(0), const glm::vec3& lightDir = glm::vec3(0)) override;

	GLuint GetTexutreID() override
	{
		return m_depthTexture;
	}
	glm::mat4 GetMVP() override
	{
		return m_depthVP;
	}

private:
	static const glm::vec3 quad_vBufferData[];

	glm::mat4 m_depthProjMatrix = glm::ortho<float>(-10.0f, 10.0f, -10.0f, 10.0f, -20, 20);
	glm::mat4 m_depthVP = glm::mat4(1.f);

	GLuint m_quad_vertexbuffer;
	GLuint m_frameBuffer = 0;
	GLuint m_depthTexture = 0;
	GLuint m_depthRenderBuffer = 0;
	GLuint m_MVPID = 0;
	GLuint m_texID = 0;
	GLuint m_timeID = 0;
	GLenum m_drawBuffers[1];

	int m_textureWidth = 1024;
	int m_textureHeight = 1024;
	int m_textureX = 0;
	int m_textureY = 0;


};

#endif