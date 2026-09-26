#version 460 core
// Generic coloured line/point vertices in world space.
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColour;

uniform mat4 uViewProj;

out vec4 vColour;

void main()
{
    vColour = aColour;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
