//GLSL VERTEX SHADER
#version 330 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;
layout(location = 2) in vec3 normal;

uniform mat4 MVP;
uniform mat4 DepthMVP;
uniform mat4 V;
uniform mat4 M;
uniform mat3 transInverseM;
//uniform vec3 LightPosition_worldspace;

//out vec4 vert_color;
//out vec3 vert_normal;
//out vec3 Position_worldspace;
//out vec3 Normal_cameraspace;
//out vec3 EyeDirection_cameraspace;
//out vec3 LightDirection_cameraspace;
//out vec4 ShadowCoord;

out vec3 fragPos;
out vec3 fragNormal;
out vec4 fragColor;
out vec4 shadowCoords;

void main()
{
	
	gl_Position = MVP * vec4(position,1);
	shadowCoords = DepthMVP * M * vec4(position,1);
	fragPos = position;
	fragNormal = transInverseM  * normal;
	fragColor = vec4(color, 0);

	////Position of the vertex, in worldspace : M * position
	//Position_worldspace = (M * vec4(position,1)).xyz;

	////Vector that goes from the vertex to the camera, in camera space.
	////In camera space, the camera is at the origin (0,0,0).
	//vec3 vertexPosition_cameraspace = ( V * M * vec4(position,1)).xyz;
	//EyeDirection_cameraspace = vec3(0,0,0) - vertexPosition_cameraspace;

	////Vector that goes from the vertex to the light, in camera space. M is ommited because it's identity.
	//vec3 LightPosition_cameraspace = ( V * vec4(LightPosition_worldspace,1)).xyz;
	//LightDirection_cameraspace = LightPosition_cameraspace;// + EyeDirection_cameraspace;
	
	////Normal of the the vertex, in camera space
	//Normal_cameraspace = ( V * M * vec4(normal,0)).xyz; // Only correct if ModelMatrix does not scale the model ! Use its inverse transpose if not.

	//vert_color = vec4(color,1);
	//vert_normal = normal;
}