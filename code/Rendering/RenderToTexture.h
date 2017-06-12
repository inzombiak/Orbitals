#ifndef RENDER_TO_TEXTURE_H
#define RENDER_TO_TEXTURE_H

#include "IRendererType.h"

class RenderToTexture : public IRendererType
{
public:
	RenderToTexture()
	{
		m_type = Orbitals::DrawingType::TEXTURE;
	}

	void Init(GLuint preDrawProgram, GLuint drawProgram) override;
	GLuint Draw(const glm::mat4& proj, const glm::mat4& view, const glm::vec3& lightPos = glm::vec3(0), const glm::vec3& lightDir = glm::vec3(0)) override;
	GLuint PreDraw(const glm::mat4& proj, const glm::mat4& view, const glm::vec3& lightPos = glm::vec3(0), const glm::vec3& lightDir = glm::vec3(0)) override;

	GLuint GetTexutreID() override
	{
		return m_renderedTexture;
	}
	glm::mat4 GetMVP() override
	{
		return glm::mat4(1.f);
	}
private:

	static const glm::vec3 quad_vBufferData[];

	GLuint m_quad_vertexbuffer;
	GLuint m_frameBuffer = 0;
	GLuint m_renderedTexture = 0;
	GLuint m_depthRenderBuffer = 0;
	GLuint m_texID = 0;
	GLuint m_timeID = 0;
	GLenum m_drawBuffers[1];
	int m_textureWidth = 512;
	int m_textureHeight = 320;
	int m_textureX = 0;
	int m_textureY = 0;
};

#endif