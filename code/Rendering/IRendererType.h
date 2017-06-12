#ifndef I_RENDERER_TYPE_H
#define I_RENDERER_TYPE_H

#define GLEW_STATIC
#include <GL\glew.h>

#include <string>
#include <iostream>

#include "..\Utilities\HelperFunctions.h"
#include "../Utilities/OrbitalsDefs.h"

#include "RenderComponent.h"

class IRendererType
{
public:
	virtual ~IRendererType()
	{  }

	virtual void Init(GLuint preDrawProgram, GLuint drawProgram) = 0;
	//Returns ID of program
	virtual GLuint Draw(const glm::mat4& proj = glm::mat4(1.f), const glm::mat4& view = glm::mat4(1.f), const glm::vec3& lightPos = glm::vec3(0), const glm::vec3& lightDir = glm::vec3(0)) = 0;
	virtual GLuint PreDraw(const glm::mat4& proj = glm::mat4(1.f), const glm::mat4& view = glm::mat4(1.f), const glm::vec3& lightPos = glm::vec3(0), const glm::vec3& lightDir = glm::vec3(0))
	{
		//Optional
		return -1;
	}
	Orbitals::DrawingType GetType() const
	{
		return m_type;
	}

	//Need to rethink this whole rendering thing, its more interconnected than I thought
	virtual GLuint GetTexutreID() = 0;
	virtual glm::mat4 GetMVP() = 0;

protected:

	GLuint m_preProgID;
	GLuint m_progID;
	Orbitals::DrawingType m_type;
};

#endif