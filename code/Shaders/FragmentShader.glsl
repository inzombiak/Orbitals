//GLSL FRAGMENT SHADER
#version 330 core

out vec4 color;
in vec4 vert_color;

void main()
{
    color = vert_color;
}