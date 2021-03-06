//
#version 330

out vec4 FragColor;

in vec3 fragPos;
in vec3 fragNormal;
in vec4 fragColor;
in vec4 shadowCoords;

uniform sampler2DShadow shadowMap;

uniform vec4 lightPos;
uniform vec3 viewPos;

vec2 poissonDisk[16] = vec2[]( 
   vec2( -0.94201624, -0.39906216 ), 
   vec2( 0.94558609, -0.76890725 ), 
   vec2( -0.094184101, -0.92938870 ), 
   vec2( 0.34495938, 0.29387760 ), 
   vec2( -0.91588581, 0.45771432 ), 
   vec2( -0.81544232, -0.87912464 ), 
   vec2( -0.38277543, 0.27676845 ), 
   vec2( 0.97484398, 0.75648379 ), 
   vec2( 0.44323325, -0.97511554 ), 
   vec2( 0.53742981, -0.47373420 ), 
   vec2( -0.26496911, -0.41893023 ), 
   vec2( 0.79197514, 0.19090188 ), 
   vec2( -0.24188840, 0.99706507 ), 
   vec2( -0.81409955, 0.91437590 ), 
   vec2( 0.19984126, 0.78641367 ), 
   vec2( 0.14383161, -0.14100790 ) 
);

float ShadowCalculation(vec4 fragPosLightSpace, float normLightDirDot)
{

	//Bias to avoid shadow acne
	//normalLightDirDot - dot product of normal and light direction
	float bias = 0.00015 *tan(acos(normLightDirDot)); // cosTheta is dot( n,l ), clamped between 0 and 1
	bias = clamp(bias, 0,0.01);
	vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
	//projCoords = projCoords * 0.5 + 0.5;
//	projCoords.z += bias;
	float shadow = 1.0;
	for (int i=0;i<4;i++)
	{
		projCoords.xy += poissonDisk[i]/700.0;
		if ( texture( shadowMap, projCoords )  >  (projCoords.z-bias) / fragPosLightSpace.w)
		{
			shadow-=0.2;
		}
	}

	//float closestDepth = texture(shadowMap, projCoords.xyz);  
	//float currentDepth = projCoords.z;  
	//float shadow = closestDepth;//currentDepth> closestDepth  ? 1.0 : 0.0;  
	 //if(projCoords.z > 1.0)
        //shadow = 0.0;
	return shadow;
}

void main()
{
	//Color from vertex shader
	vec3 color = fragColor.xyz;
	//Color of the light
    vec3 lightColor = vec3(1.0);

	//Vertex normal
    vec3 normal = normalize(fragNormal);
	vec3 lightDir = normalize(lightPos.xyz - lightPos.w * fragPos);

	vec3 viewDir = normalize(viewPos - fragPos);
	vec3 halfwayDir = normalize(lightDir + viewDir);  

    // ambient
    vec3 ambient = 0.15 * color;
    // diffuse
    float diff = clamp(dot(lightDir, normal), 0.0, 1.0);
    vec3 diffuse = diff * lightColor;
    // specular
    float spec = 0.0;
	//Specular modifier
	//We use the Blinn-Phong model, where the angle between the normal vectory and
	//the halfway vector(between light and viewer) is used
    spec = pow(max(dot(normal, halfwayDir), 0.0), 64.0);
	//Specular color
    vec3 specular = spec * lightColor;    
    // calculate shadow
    float shadow = ShadowCalculation(shadowCoords, diff);     

	
    vec3 lighting = (ambient + (1.0 - shadow) * (diffuse + specular)) * color;    
    
    FragColor = vec4(lighting, 1.0);
}


// OLD CODE FOR ONLY LIGHTING
/*
vec2 poissonDisk[16] = vec2[]( 
   vec2( -0.94201624, -0.39906216 ), 
   vec2( 0.94558609, -0.76890725 ), 
   vec2( -0.094184101, -0.92938870 ), 
   vec2( 0.34495938, 0.29387760 ), 
   vec2( -0.91588581, 0.45771432 ), 
   vec2( -0.81544232, -0.87912464 ), 
   vec2( -0.38277543, 0.27676845 ), 
   vec2( 0.97484398, 0.75648379 ), 
   vec2( 0.44323325, -0.97511554 ), 
   vec2( 0.53742981, -0.47373420 ), 
   vec2( -0.26496911, -0.41893023 ), 
   vec2( 0.79197514, 0.19090188 ), 
   vec2( -0.24188840, 0.99706507 ), 
   vec2( -0.81409955, 0.91437590 ), 
   vec2( 0.19984126, 0.78641367 ), 
   vec2( 0.14383161, -0.14100790 ) 
);
// Interpolated values from the vertex shaders
in vec4 vert_color;
in vec3 vert_normal;
in vec3 Position_worldspace;
in vec3 Normal_cameraspace;
in vec3 EyeDirection_cameraspace;
in vec3 LightDirection_cameraspace;
in vec4 ShadowCoord;

// Ouput data
layout(location = 0) out vec4 FragColor;

uniform mat4 MV;
uniform vec3 LightPosition_worldspace;
uniform sampler2D shadowMap;

void main()
{
	//Light emission properties
	//You probably want to put them as uniforms
	vec3 LightColor = vec3(1,1,1);
	float LightPower = 1.f;
	
	//Material properties
	vec3 MaterialDiffuseColor = vec3(vert_color[0], vert_color[1], vert_color[2]);
	vec3 MaterialAmbientColor = vec3(0.1,0.1,0.1) * MaterialDiffuseColor;
	vec3 MaterialSpecularColor = vec3(0.3,0.3,0.3);

	//Distance to the light
	float distance = length( LightPosition_worldspace - Position_worldspace );

	//Normal of the computed fragment, in camera space
	vec3 n = normalize( Normal_cameraspace );
	//Direction of the light (from the fragment to the light)
	vec3 l = normalize( LightDirection_cameraspace );
	//Cosine of the angle between the normal and the light direction, 
	//clamped above 0
	//- light is at the vertical of the triangle -> 1
	//- light is perpendicular to the triangle -> 0
	//- light is behind the triangle -> 0
	float cosTheta = clamp( dot( n,l ), 0.0, 1.0);
	
	//Eye vector (towards the camera)
	vec3 E = normalize(EyeDirection_cameraspace);
	//Direction in which the triangle reflects the light
	vec3 R = reflect(-l,n);
	//Cosine of the angle between the Eye vector and the Reflect vector,
	//clamped to 0
	//- Looking into the reflection -> 1
	//- Looking elsewhere -> < 1
	float cosAlpha = clamp( dot( E,R ), 0.0, 1.0);
	
	//float bias = 0.005;//*tan(acos(cosTheta)); // cosTheta is dot( n,l ), clamped between 0 and 1
	//bias = clamp(bias, 0,0.01);
	float visibility = 1;
	if ( texture( shadowMap, ShadowCoord.xy ).z < ShadowCoord.z)
	{
		visibility = 0.1;
	}
	//for(int i = 0; i < 4; ++i)
	//{
	//	int index= i;
	//	//Directional
	//	visibility -= 0.2*(1.0-texture( shadowMap, vec3((ShadowCoord.xy + poissonDisk[index]/700.0)/ShadowCoord.w, (ShadowCoord.z-bias)/ShadowCoord.w) ));
	//	//Spot
	//	//if ( texture( shadowMap, ShadowCoord.xyz/ShadowCoord.w)  <  (ShadowCoord.z - bias)/ShadowCoord.w )
	//	//	visibility -= 0.2;
	//}

	vec3 color =
    //Ambient : simulates indirect lighting
	MaterialAmbientColor +
   //Diffuse : "color" of the object
   visibility * MaterialDiffuseColor * LightColor * LightPower  * cosTheta + /// (distance*distance)  +
   //Specular : reflective highlight, like a mirror			//
	visibility * MaterialSpecularColor * LightColor * LightPower * pow(cosAlpha, 5.0);// / (distance*distance);
	FragColor = vec4(color, 1);
}
*/