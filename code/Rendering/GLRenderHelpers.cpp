#include "GLRenderHelpers.h"

#include <map>
#include <string>


bool is_near(float v1, float v2)
{
	return fabs(v1 - v2) < 0.01f;
}

struct PackedVertex{
	glm::vec3 position;
	//glm::vec2 uv;
	glm::vec3 normal;
	//bool operator
};

struct ComparePackedVertices
{
	bool operator()(const PackedVertex& a, const PackedVertex& b) const
	{
		if (a.position.z < b.position.z)
			return true;
		else if (a.position.z > b.position.z)
			return false;

		if (a.position.y < b.position.y)
			return true;
		else if (a.position.y > b.position.y)
			return false;

		if (a.position.x < b.position.x)
			return true;
		else if (a.position.x > b.position.x)
			return false;

		if (a.normal.z < b.normal.z)
			return true;
		else if (a.normal.z > b.normal.z)
			return false;

		if (a.normal.y < b.normal.y)
			return true;
		else if (a.normal.y > b.normal.y)
			return false;

		if (a.normal.x < b.normal.x)
			return true;
		else if (a.normal.x > b.normal.x)
			return false;

		return false;
	}
};

bool getSimilarVertexIndex_fast(
	PackedVertex & packed,
	std::map<PackedVertex, unsigned short, ComparePackedVertices> & VertexToOutIndex,
	unsigned short & result
	)
{
	std::map<PackedVertex, unsigned short>::iterator it = VertexToOutIndex.find(packed);
	if (it == VertexToOutIndex.end())
	{
		return false;
	}
	else
	{
		result = it->second;
		return true;
	}
}



void IndexVBO(
	std::vector<glm::vec3> & in_vertices,
	//std::vector<glm::vec2> & in_uvs,
	std::vector<glm::vec3> & in_normals,

	std::vector<unsigned short> & out_indices,
	std::vector<glm::vec3> & out_vertices,
	//std::vector<glm::vec2> & out_uvs,
	std::vector<glm::vec3> & out_normals
	)
{
	std::map<PackedVertex, unsigned short, ComparePackedVertices> VertexToOutIndex;

//	printf(in_vertices.size());
	// For each input vertex
	for (unsigned int i = 0; i < in_vertices.size(); i++)
	{

		//PackedVertex packed = { in_vertices[i], in_uvs[i], in_normals[i] };
		PackedVertex packed = { in_vertices[i], in_normals[i] };

		// Try to find a similar vertex in out_XXXX
		unsigned short index;
		bool found = getSimilarVertexIndex_fast(packed, VertexToOutIndex, index);

		if (found)
		{ // A similar vertex is already in the VBO, use it instead !
			out_indices.push_back(index);
			printf("Found");
		}
		else
		{ // If not, it needs to be added in the output data.
			out_vertices.push_back(in_vertices[i]);
			//out_uvs.push_back(in_uvs[i]);
			out_normals.push_back(in_normals[i]);
			unsigned short newindex = (unsigned short)out_vertices.size() - 1;
			out_indices.push_back(newindex);
			VertexToOutIndex[packed] = newindex;
		}
	}
}