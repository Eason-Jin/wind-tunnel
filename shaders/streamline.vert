#version 460 core
// Streamline vertex: world position plus the local speed and cumulative arc
// length (from the seed) used for colouring and the dash animation.
layout(location = 0) in vec3 aPos;
layout(location = 1) in float aSpeed;
layout(location = 2) in float aArc;

uniform mat4 uViewProj;

out VS_OUT {
    float speed;
    float arc;
} vOut;

void main()
{
    gl_Position = uViewProj * vec4(aPos, 1.0);
    vOut.speed = aSpeed;
    vOut.arc = aArc;
}
