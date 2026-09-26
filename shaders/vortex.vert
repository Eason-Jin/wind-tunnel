#version 460 core
// Vortex isosurface vertex: world-space position + normal (from the Q
// gradient) + a per-vertex colour baked on the CPU (swirl direction or
// velocity magnitude, already run through a colormap).
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;

uniform mat4 uViewProj;

out vec3 vWorldPos;
out vec3 vNormal;
out vec3 vColor;

void main()
{
    vWorldPos = aPos;
    vNormal = aNormal;
    vColor = aColor;
    gl_Position = uViewProj * vec4(aPos, 1.0);
}
