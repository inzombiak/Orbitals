#ifndef EVENT_DEFS_H
#define EVENT_DEFS_H

#include <vector>

#include <glm\glm.hpp>

class ICelestialObject;
struct RenderCompCreationData
{
	ICelestialObject* owner = 0;

	int id = -1;

	std::vector<unsigned short> indicies;
	std::vector<unsigned short> indiciesClean;
	std::vector<glm::vec3> verticesClean;
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec3> color;
	std::vector<glm::vec3> normals;
	unsigned int drawType = 0;
};

#endif