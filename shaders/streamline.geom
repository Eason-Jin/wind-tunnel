#version 460 core
// Expands each independent line segment (GL_LINES) into a camera-facing,
// constant-pixel-width quad, since core-profile OpenGL caps glLineWidth at
// 1px. The offset is computed in screen space so the ribbon keeps a fixed
// width in pixels regardless of depth.
layout(lines) in;
layout(triangle_strip, max_vertices = 4) out;

in VS_OUT {
    float speed;
    float arc;
} vIn[];

out GS_OUT {
    float speed;
    float arc;
    float widthT; // -1..1 across the ribbon, for fake tube shading
} vOut;

uniform vec2 uViewport;
uniform float uWidthPx;

void emit(vec4 clipPos, vec2 offsetClip, float speed, float arc, float widthT)
{
    gl_Position = vec4(clipPos.xy + offsetClip, clipPos.z, clipPos.w);
    vOut.speed = speed;
    vOut.arc = arc;
    vOut.widthT = widthT;
    EmitVertex();
}

void main()
{
    vec4 p0 = gl_in[0].gl_Position;
    vec4 p1 = gl_in[1].gl_Position;

    vec2 halfViewport = 0.5 * uViewport;
    vec2 s0 = (p0.xy / p0.w) * halfViewport;
    vec2 s1 = (p1.xy / p1.w) * halfViewport;

    vec2 dir = s1 - s0;
    float len = length(dir);
    vec2 n = len > 1e-6 ? vec2(-dir.y, dir.x) / len : vec2(1.0, 0.0);
    n *= 0.5 * uWidthPx;

    // Screen-space offset back to a clip-space delta (undo the /w that
    // produced screen space, per endpoint).
    vec2 o0 = (n / halfViewport) * p0.w;
    vec2 o1 = (n / halfViewport) * p1.w;

    emit(p0, o0, vIn[0].speed, vIn[0].arc, -1.0);
    emit(p0, -o0, vIn[0].speed, vIn[0].arc, 1.0);
    emit(p1, o1, vIn[1].speed, vIn[1].arc, -1.0);
    emit(p1, -o1, vIn[1].speed, vIn[1].arc, 1.0);
    EndPrimitive();
}
