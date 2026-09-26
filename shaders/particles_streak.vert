#version 460 core
// Two vertices per particle (previous position -> current position), drawn
// as GL_LINES. Reads straight from the SSBO via gl_VertexID; no VBO.
struct Particle {
    vec4 posAge;   // xyz = world position, w = age (s)
    vec4 prevLife; // xyz = previous position, w = lifetime (s)
};

layout(std430, binding = 0) readonly buffer ParticleBuffer {
    Particle particles[];
};

uniform sampler3D uFlow;
uniform vec3 uGridMin;
uniform vec3 uGridSize;
uniform float uMaxSpeed;

uniform mat4 uViewProj;
uniform int uColorMode; // 0 = speed colormap, 1 = smoke colour
uniform vec3 uSmokeColor;
uniform float uOpacity;
uniform sampler1D uColormap;

out vec4 vColour;

const float kFadeTime = 0.15; // seconds

void main()
{
    uint idx = uint(gl_VertexID) >> 1;
    bool head = (gl_VertexID & 1) == 1;

    Particle p = particles[idx];
    vec3 headPos = p.posAge.xyz;
    vec3 tailPos = p.prevLife.xyz;
    vec3 pos = head ? headPos : tailPos;

    float age = p.posAge.w;
    float lifetime = max(p.prevLife.w, 1e-4);

    gl_Position = uViewProj * vec4(pos, 1.0);

    vec3 tc = (headPos - uGridMin) / uGridSize;
    vec3 vel = textureLod(uFlow, tc, 0.0).rgb;
    float t = clamp(length(vel) / max(uMaxSpeed, 1e-6), 0.0, 1.0);

    vec3 baseColour = (uColorMode == 0) ? textureLod(uColormap, t, 0.0).rgb : uSmokeColor;
    float fadeIn = clamp(age / kFadeTime, 0.0, 1.0);
    float fadeOut = clamp((lifetime - age) / kFadeTime, 0.0, 1.0);
    float fade = min(fadeIn, fadeOut);
    float endFade = head ? 1.0 : 0.35; // dimmer tail gives the streak a direction

    vColour = vec4(baseColour, uOpacity * fade * endFade);
}
