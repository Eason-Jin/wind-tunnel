#version 460 core
// Body mesh vertex: world-space position + normal, plus an optional scalar
// (surface pressure coefficient) used for colouring.
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in float aScalar;

uniform mat4 uViewProj;
uniform vec4 uClipPlane; // section cut: fragments with dot < 0 are clipped (only while GL_CLIP_DISTANCE0 is enabled)

out vec3 vWorldPos;
out vec3 vNormal;
out float vScalar;

void main()
{
    vWorldPos = aPos;
    vNormal = aNormal;
    vScalar = aScalar;
    gl_Position = uViewProj * vec4(aPos, 1.0);
    gl_ClipDistance[0] = dot(uClipPlane, vec4(aPos, 1.0));
}
