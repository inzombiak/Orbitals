#include "ObjectCreatorFunctions.h"

#include "../Rendering/GLRenderHelpers.h"
#include "../Physics/PhysicsDefs.h"
#include "..\Utilities\HelperFunctions.h"

#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>


EDCreateObject* ObjectCreators::CreateSphereEventData(IDefaultShapeData* data)
{
	if (data->GetType() != Sphere)
		return 0;

	SphereShapeData* sData = static_cast<SphereShapeData*>(data);
	ObjectCreationData* ballData = new ObjectCreationData();
	ballData->name = "ball";
	ballData->renderCompData->color.push_back(data->color);
	ballData->renderCompData->drawType = GL_TRIANGLES;

	std::vector<glm::vec3> vertices;
	std::vector<unsigned short> faceIndices;
	std::vector<unsigned short> cleanIndices;
	std::vector<glm::vec3> normals;
	std::vector<glm::vec3> cleanVert;
	CreateSphere(glm::vec3(0, 0, 0), sData->radius, vertices, normals, cleanVert, cleanIndices);

	IndexVBO(vertices, normals,
		ballData->renderCompData->indicies, ballData->renderCompData->vertices, ballData->renderCompData->normals);
	ballData->renderCompData->verticesClean = cleanVert;
	ballData->renderCompData->indiciesClean = cleanIndices;

	PhysicsDefs::SphereCreationData* scd = new PhysicsDefs::SphereCreationData();
	scd->sphereRadius = sData->radius;

	scd->rbci.mass = 1.f;
	scd->rbci.linearDamping = 0.f;
	scd->rbci.angularDamping = 0.f;
	scd->rbci.friction = 0.3f;
	scd->rbci.rollingFriction = 0.3f;
	scd->rbci.resititution = 0.5f;
	scd->rbci.enableGravity - true;

	glm::mat4 translationMat = glm::translate(data->position);
	glm::mat4 scalingMat = glm::scale(data->scale);
	glm::mat4 rotationMat = glm::toMat4(glm::quat(data->rotation));
	scd->rbci.transform = translationMat * rotationMat * scalingMat;
	scd->rbci.localInertia = glm::vec3(0, 0, 0); //TODO: INCORRECT, add calculator funciton;

	ballData->rigidBodyData = scd;

	//ballData->renderCompData->normals = normals;
	auto eventData = new EDCreateObject(ballData);
	
	return eventData;
}

EDCreateObject* ObjectCreators::CreateBoxEventData(IDefaultShapeData* data)
{
	if (data->GetType() != Box)
		return 0;

	BoxShapeData* bData = static_cast<BoxShapeData*>(data);

	ObjectCreationData* boxData = new ObjectCreationData();
	boxData->name = "box";
	boxData->renderCompData->color.push_back(data->color);
	boxData->renderCompData->drawType = GL_TRIANGLES;


	std::vector<glm::vec3> vertices;
	std::vector<unsigned short> faceIndices;
	std::vector<unsigned short> cleanIndices;
	std::vector<glm::vec3> normals;
	std::vector<glm::vec3> cleanVert;
	CreateBox(glm::vec3(0, 0, 0), bData->extents.x, bData->extents.y, bData->extents.z, vertices, normals, cleanVert, cleanIndices);
	IndexVBO(vertices, normals,
		boxData->renderCompData->indicies, boxData->renderCompData->vertices, boxData->renderCompData->normals);

	PhysicsDefs::BoxCreationData* bcd = new PhysicsDefs::BoxCreationData();
	bcd->boxDimensions = bData->extents;

	bcd->rbci.mass = 0.f;
	bcd->rbci.linearDamping = 0.f;
	bcd->rbci.angularDamping = 0.f;
	bcd->rbci.friction = 0.3f;
	bcd->rbci.rollingFriction = 0.3f;
	bcd->rbci.resititution = 1.f;
	bcd->rbci.enableGravity - true;

	glm::mat4 translationMat = glm::translate(data->position);
	glm::mat4 scalingMat = glm::scale(data->scale);
	glm::mat4 rotationMat = glm::toMat4(glm::quat(data->rotation));
	bcd->rbci.transform = translationMat * rotationMat * scalingMat;
	bcd->rbci.localInertia = glm::vec3(0, 0, 0); //TODO: INCORRECT, add calculator funciton;

	boxData->rigidBodyData = bcd;

	boxData->renderCompData->verticesClean = cleanVert;
	boxData->renderCompData->indiciesClean = cleanIndices;
	auto eventData = new EDCreateObject(boxData);

	return eventData;
}
