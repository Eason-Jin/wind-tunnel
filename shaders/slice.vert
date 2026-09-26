#version 460 core
// Slice plane quad, in world space (see FlowTextures for the flow grid frame).
layout(location = 0) in vec3 aPos;

uniform mat4 uViewProj;

out vec3 vWorldPos;

void main()
{
    vWorldPos = aPos;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
