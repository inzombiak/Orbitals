#include "Shadowmap.h"

void Shadowmap::Init(GLuint preDrawProgram, GLuint drawProgram)
{
	m_preProgID = preDrawProgram;
	m_progID = drawProgram;
	glGenFramebuffers(1, &m_frameBuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, m_frameBuffer);

	glGenTextures(1, &m_depthTexture);
	glBindTexture(GL_TEXTURE_2D, m_depthTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT16, m_textureWidth, m_textureHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, 0);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);

	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, m_depthTexture, 0);

	glDrawBuffer(GL_NONE);
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		std::cout << "Error setting up depth buffer" << std::endl;
		//TODO: ERROR
	}

	m_MVPID = glGetUniformLocation(m_preProgID, "MVP");
	glGenBuffers(1, &m_quad_vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, m_quad_vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * 6, quad_vBufferData, GL_STATIC_DRAW);

	m_texID = glGetUniformLocation(m_progID, "renderedTexture");
	m_timeID = glGetUniformLocation(m_progID, "time");
}

GLuint Shadowmap::PreDraw(const glm::mat4& proj, const glm::mat4& view, const glm::vec3& lightPos, const glm::vec3& lightDir)
{
	
	//Spot light
	if (glm::dot(lightPos, lightPos) != 0)
	{
		glm::mat4 depthProjectionMatrix = glm::perspective<float>(45.0f, 1.0f, 2.0f, 50.0f);
		glm::mat4 depthViewMatrix = glm::lookAt(lightPos, lightPos -lightDir, glm::vec3(0, 1, 0));
		m_depthVP = depthProjectionMatrix * depthViewMatrix;
	}
	else
	{
		//Directional light
		glm::mat4 depthViewMatrix = glm::lookAt(lightDir, glm::vec3(0, 0, 0), glm::vec3(0, 1, 0));
		m_depthVP = m_depthProjMatrix * depthViewMatrix;
	}

	

	glUseProgram(m_preProgID);
	glBindFramebuffer(GL_FRAMEBUFFER, m_frameBuffer);
	glCullFace(GL_FRONT);
	glViewport(m_textureX, m_textureY, m_textureWidth, m_textureHeight);

	return m_preProgID;
}

GLuint Shadowmap::Draw(const glm::mat4& proj, const glm::mat4& view, const glm::vec3& lightPos, const glm::vec3& lightDir)
{
	GLuint err;

	//Draw frame buffer to screen
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glUseProgram(m_progID);
	glViewport(m_textureX, m_textureY, m_textureWidth/3, m_textureHeight/3);
	glActiveTexture(GL_TEXTURE0);
	//glBindTexture(GL_TEXTURE_2D, m_renderedTexture);

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

const glm::vec3 Shadowmap::quad_vBufferData[] =
{
	glm::vec3(-1.0f, -1.0f, 0.0f),
	glm::vec3(1.0f, -1.0f, 0.0f),
	glm::vec3(-1.0f, 1.0f, 0.0f),
	glm::vec3(-1.0f, 1.0f, 0.0f),
	glm::vec3(1.0f, -1.0f, 0.0f),
	glm::vec3(1.0f, 1.0f, 0.0f)
};