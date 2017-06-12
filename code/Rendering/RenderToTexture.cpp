#include "RenderToTexture.h"

void RenderToTexture::Init(GLuint preDrawProgram, GLuint drawProgram)
{
	m_preProgID = preDrawProgram;
	m_progID = drawProgram;

	glGenFramebuffers(1, &m_frameBuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, m_frameBuffer);

	glGenTextures(1, &m_renderedTexture);
	glBindTexture(GL_TEXTURE_2D, m_renderedTexture);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_textureWidth, m_textureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, 0);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

	glGenRenderbuffers(1, &m_depthRenderBuffer);
	glBindRenderbuffer(GL_RENDERBUFFER, m_depthRenderBuffer);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, m_textureWidth, m_textureHeight);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_depthRenderBuffer);

	glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, m_renderedTexture, 0);

	m_drawBuffers[0] = { GL_COLOR_ATTACHMENT0 };
	glDrawBuffers(1, m_drawBuffers);
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		std::cout << "Error setting up depth buffer" << std::endl;
		//TODO: ERROR
	}

	glGenBuffers(1, &m_quad_vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_quad_vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * 6, quad_vBufferData, GL_STATIC_DRAW);

	m_texID = glGetUniformLocation(m_progID, "renderedTexture");
	m_timeID = glGetUniformLocation(m_progID, "time");

}

GLuint RenderToTexture::PreDraw(const glm::mat4& proj, const glm::mat4& view, const glm::vec3& lightPos, const glm::vec3& lightDir)
{
	glUseProgram(m_preProgID);
	glBindFramebuffer(GL_FRAMEBUFFER, m_frameBuffer);
	glViewport(m_textureX, m_textureY, m_textureWidth, m_textureHeight);

	return m_preProgID;
}

GLuint RenderToTexture::Draw(const glm::mat4& proj, const glm::mat4& view, const glm::vec3& lightPos, const glm::vec3& lightDir)
{
	GLuint err;

	//Draw frame buffer to screen
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glUseProgram(m_progID);
	glViewport(m_textureX, m_textureY, m_textureWidth, m_textureHeight);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_renderedTexture);

	// Set our "renderedTexture" sampler to user Texture Unit 0
	glUniform1i(m_texID, 0);
	glUniform1f(m_timeID, (float)0.16*10.0f);
	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, m_quad_vertexbuffer);
	glVertexAttribPointer(
		0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
		);

	glDrawArrays(GL_TRIANGLES, 0, 6);
	glDisableVertexAttribArray(0);
	while ((err = glGetError()) != GL_NO_ERROR)
	{
		std::cerr << "OpenGL error: " << err << std::endl;
	}

	return m_progID;
}

const glm::vec3 RenderToTexture::quad_vBufferData[] =
{
	glm::vec3(-1.0f, -1.0f, 0.0f),
	glm::vec3(1.0f, -1.0f, 0.0f),
	glm::vec3(-1.0f, 1.0f, 0.0f),
	glm::vec3(-1.0f, 1.0f, 0.0f),
	glm::vec3(1.0f, -1.0f, 0.0f),
	glm::vec3(1.0f, 1.0f, 0.0f)
};