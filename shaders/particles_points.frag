#version 460 core
// Soft round point sprite.
in vec4 vColour;
out vec4 FragColor;

void main()
{
    vec2 d = gl_PointCoord - vec2(0.5);
    float r2 = dot(d, d);
    if (r2 > 0.25)
        discard;
    float falloff = smoothstep(0.25, 0.0, r2);
    FragColor = vec4(vColour.rgb, vColour.a * falloff);
}
