//GLSL VERTEX SHADER
#version 330

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 color;

uniform mat4 MVP;

out vec4 vert_color;

void main()
{
	gl_PointSize = 5.f;
	gl_Position = MVP * vec4(position,1);
	vert_color = vec4(color,1);
}