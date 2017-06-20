#version 330 core

in vec2 UV;

out vec3 color;

uniform sampler2D renderedTexture;
uniform float time;

float near_plane = 2;
float far_plane = 50;

float LinearizeDepth(float depth)
{
    float z = depth * 2.0 - 1.0; // Back to NDC 
    return (2.0 * near_plane * far_plane) / (far_plane + near_plane - z * (far_plane - near_plane));
}

void main()
{
    //color = texture( renderedTexture, UV + 0.0005*vec2( sin(time+1024.0*UV.x),cos(time+768.0*UV.y)) ).xyz;

	float depthValue = texture(renderedTexture, UV).r;
	color = vec3(LinearizeDepth(depthValue) / far_plane);
}