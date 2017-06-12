//GLSL VERTEX SHADER
#version 330 core

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;
layout(location = 2) in vec3 normal;

uniform mat4 MVP;
uniform mat4 DepthMVP;
uniform mat4 V;
uniform mat4 M;
uniform vec3 LightPosition_worldspace;

out vec4 vert_color;
out vec3 Position_worldspace;
out vec3 Normal_cameraspace;
out vec3 EyeDirection_cameraspace;
out vec3 LightDirection_cameraspace;
out vec4 ShadowCoord;

void main()
{
	
	gl_PointSize = 5.0;
	gl_Position = MVP * vec4(position,1);
	ShadowCoord = DepthMVP * vec4(position,1);
	if(length(ShadowCoord) < 0.5)
		gl_Position = vec4(0, 0, 0, 0);
	//Position of the vertex, in worldspace : M * position
	Position_worldspace = (M * vec4(position,1)).xyz;

	// Vector that goes from the vertex to the camera, in camera space.
	// In camera space, the camera is at the origin (0,0,0).
	vec3 vertexPosition_cameraspace = ( V * M * vec4(position,1)).xyz;
	EyeDirection_cameraspace = vec3(0,0,0) - vertexPosition_cameraspace;

	// Vector that goes from the vertex to the light, in camera space. M is ommited because it's identity.
	vec3 LightPosition_cameraspace = ( V * vec4(LightPosition_worldspace,1)).xyz;
	LightDirection_cameraspace = LightPosition_cameraspace + EyeDirection_cameraspace;
	
	// Normal of the the vertex, in camera space
	Normal_cameraspace = ( V * M * vec4(normal,0)).xyz; // Only correct if ModelMatrix does not scale the model ! Use its inverse transpose if not.

	vert_color = vec4(color,1);
}