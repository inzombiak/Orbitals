#version 330 core

in vec2 UV;

out vec3 color;

uniform sampler2D renderedTexture;
uniform float time;

void main()
{
    color = texture( renderedTexture, UV + 0.0005*vec2( sin(time+1024.0*UV.x),cos(time+768.0*UV.y)) ).xyz;
}