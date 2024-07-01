#version 430 core

uniform mat4 modelMatrix 	= mat4(1.0f);
uniform mat4 viewMatrix 	= mat4(1.0f);
uniform mat4 projMatrix 	= mat4(1.0f);
uniform mat4 shadowMatrix 	= mat4(1.0f);

layout(location = 0) in vec3 position;
layout(location = 1) in vec4 colour;
layout(location = 2) in vec2 texCoord;
layout(location = 3) in vec3 normal;
layout(location = 7) in vec3 instancedParticlePosition;

uniform vec4 		objectColour = vec4(1,1,1,1);

uniform bool hasVertexColours = false;

struct Particle {
    vec3 Position;
    vec3 PredictedPosition;
    vec3 Velocity;
    vec3 PressureAcceleration;

    float density;
    float pressure;

    uint Gridhash;
};

layout(std430, binding = 0) buffer ParticleBuffer {
    Particle particles[];
};

out Vertex
{
	vec4 colour;
	vec2 texCoord;
	vec4 shadowProj;
	vec3 normal;
	vec3 worldPos;
} OUT;

mat4 UpdateModelMatrixPosition(mat4 modelMatrix, vec3 newPosition) {
    mat4 updatedModelMatrix = modelMatrix;
    updatedModelMatrix[3][0] = newPosition.x;
    updatedModelMatrix[3][1] = newPosition.y;
    updatedModelMatrix[3][2] = newPosition.z;
    return updatedModelMatrix;
}

void main(void)
{
	uint index = gl_InstanceID;

	mat4 newModelMatrix = UpdateModelMatrixPosition(modelMatrix, particles[index].Position);

	mat4 mvp 		  = (projMatrix * viewMatrix * newModelMatrix);
	mat3 normalMatrix = transpose ( inverse ( mat3 ( newModelMatrix )));

	OUT.shadowProj 	=  shadowMatrix * vec4 ( position,1);
	OUT.worldPos 	= ( newModelMatrix * vec4 ( position ,1)). xyz ;
	OUT.normal 		= normalize ( normalMatrix * normalize ( normal ));
	
	OUT.texCoord	= texCoord;
	OUT.colour		= objectColour;

	if(hasVertexColours) {
		OUT.colour		= objectColour * colour;
	}
	gl_Position		= mvp * vec4(position, 1.0);
}