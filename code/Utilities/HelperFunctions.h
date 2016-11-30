#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#define GLEW_STATIC
#include <GL\glew.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include <array>
#include <string>
#include <vector>

struct CarInfo
{
	float bodyParameters[3];
	std::vector<glm::vec3> wheelPositions;
	float wheelConnectionHeight;
	float wheelRadius;
	float wheelWidth;
};


//Shader compliation helped funcitons
//Reads a file to a std::string
std::string ReadFileToString(const char* filePath);
//Creates a GLenum type shader from a string
GLuint CreateShader(GLenum eShaderType, const std::string &strShaderFile);
//Compiles shaders into a program
GLuint CreateProgram(const std::vector<GLuint> &shaderList);

std::vector<glm::vec3> CreateCircle(glm::vec3 center, float radius);
//Sphere generation functions
//Creates an octohedron using a center and radius
std::vector<glm::vec3> CreateOctahedronWithRadius(glm::vec3 center, float radius);
//Creates an octohedron using a center and side length
std::vector<glm::vec3> CreateOctahedronWithSide(glm::vec3 center, float side);

//Splits an equilater triangle into 4 equilateral parts. First varable is an array of the 3 vertices of the triangle, 2nd variable is the number of remaining subdivisions
std::vector<glm::vec3> DivideTriangle(glm::vec3 octahedronVertices[3], int remainingSubdivisons);
//Converts an octohedron with the given faces(provided as individual vertices) to a sphere with the given center and radius. Number of subdivisions is number of times each face should be divided
std::vector<glm::vec3> OctahedronToSphere(std::vector<glm::vec3> octahedronFaces, glm::vec3 sphereCenter, float sphereRadius, int numSubdivisions,
	std::vector<glm::vec3>& vertices_out, std::vector<glm::vec3>& normals_out, std::vector<glm::vec3>& cleanVert_out, std::vector<unsigned short> &cleanIndices_out);

//Calcualte Normals
glm::vec3 CalculateNormals(const std::vector<glm::vec3>& verticies);

//Create a sphere
void CreateSphere(glm::vec3 center, float radius,
	std::vector<glm::vec3>& vertices_out, std::vector<glm::vec3>& normals_out, std::vector<glm::vec3>& cleanVert_out, std::vector<unsigned short> &cleanIndices_out);
void CreateBox(glm::vec3 center, float width, float height, float length,
	std::vector<glm::vec3>& vertices_out, std::vector<glm::vec3>& normals_out, std::vector<glm::vec3>& cleanVert_out, std::vector<unsigned short> &cleanIndices_out);
std::vector<glm::vec3> CreateCylinder(glm::vec3 center, float radius, float height);
std::vector<glm::vec3> CreateCar(CarInfo carInfo);
void CreateTriMesh(const std::vector<glm::vec3>& vertices, const std::vector<unsigned short>& triangleIndices,
	std::vector<glm::vec3>& vertices_out, std::vector<glm::vec3>& normals_out, std::vector<glm::vec3>& cleanVert_out, std::vector<unsigned short> &cleanIndices_out);

#endif