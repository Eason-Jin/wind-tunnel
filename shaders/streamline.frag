#version 460 core
in GS_OUT {
    float speed;
    float arc;
    float widthT;
} vIn;

out vec4 FragColor;

uniform sampler1D uColormap;
uniform int uColorMode; // 0 = speed, 1 = solid
uniform vec3 uSolidColour;
uniform float uSpeedMin;
uniform float uSpeedMax;
uniform bool uDashOn;
uniform float uDashFreq;
uniform float uDashSpeed;
uniform float uTime;

void main()
{
    if (uDashOn) {
        float m = fract(vIn.arc * uDashFreq - uTime * uDashSpeed);
        if (m > 0.5)
            discard;
    }

    vec3 base;
    if (uColorMode == 0) {
        float t = clamp((vIn.speed - uSpeedMin) / max(uSpeedMax - uSpeedMin, 1e-6), 0.0, 1.0);
        base = texture(uColormap, t).rgb;
    } else {
        base = uSolidColour;
    }

    // Fake round-tube shading: brighten the centre, darken the edges.
    float shade = sqrt(max(0.0, 1.0 - vIn.widthT * vIn.widthT));
    vec3 colour = base * mix(0.55, 1.0, shade);

    FragColor = vec4(colour, 1.0);
}
