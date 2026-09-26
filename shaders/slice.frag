#version 460 core
// Contour plane: samples the flow grid at the fragment's world position and
// colours it by the chosen scalar quantity, with optional iso-contour lines.

in vec3 vWorldPos;
out vec4 FragColor;

uniform sampler3D uFlow; // rgb = velocity (m/s), a = kinematic pressure
uniform sampler3D uSolid;
uniform vec3 uGridMin;
uniform vec3 uGridSize;
uniform ivec3 uGridDims;
uniform float uMaxSpeed;
uniform float uPressureMin;
uniform float uPressureMax;
uniform float uFreestream;

uniform sampler1D uColormap;

uniform int uQuantity; // 0 = speed, 1 = Cp, 2 = Ux, 3 = vorticity magnitude
uniform float uRangeMin;
uniform float uRangeMax;
uniform bool uShowSolid; // false => discard solid fragments
uniform bool uContours;
uniform int uContourCount;
uniform float uOpacity;

float scalarAt(vec3 tc, vec3 vel, float pressure)
{
    if (uQuantity == 0)
        return length(vel);
    if (uQuantity == 1)
        return pressure / max(0.5 * uFreestream * uFreestream, 1e-6);
    if (uQuantity == 2)
        return vel.x;

    // Vorticity magnitude via central differences, one texel step per axis.
    vec3 d = 1.0 / vec3(uGridDims);
    vec3 cell = uGridSize / vec3(uGridDims);
    vec3 vXp = texture(uFlow, tc + vec3(d.x, 0.0, 0.0)).rgb;
    vec3 vXm = texture(uFlow, tc - vec3(d.x, 0.0, 0.0)).rgb;
    vec3 vYp = texture(uFlow, tc + vec3(0.0, d.y, 0.0)).rgb;
    vec3 vYm = texture(uFlow, tc - vec3(0.0, d.y, 0.0)).rgb;
    vec3 vZp = texture(uFlow, tc + vec3(0.0, 0.0, d.z)).rgb;
    vec3 vZm = texture(uFlow, tc - vec3(0.0, 0.0, d.z)).rgb;

    float dVzdy = (vYp.z - vYm.z) / (2.0 * cell.y);
    float dVydz = (vZp.y - vZm.y) / (2.0 * cell.z);
    float dVxdz = (vZp.x - vZm.x) / (2.0 * cell.z);
    float dVzdx = (vXp.z - vXm.z) / (2.0 * cell.x);
    float dVydx = (vXp.y - vXm.y) / (2.0 * cell.x);
    float dVxdy = (vYp.x - vYm.x) / (2.0 * cell.y);

    vec3 curl = vec3(dVzdy - dVydz, dVxdz - dVzdx, dVydx - dVxdy);
    return length(curl);
}

void main()
{
    vec3 tc = (vWorldPos - uGridMin) / uGridSize;
    if (any(lessThan(tc, vec3(0.0))) || any(greaterThan(tc, vec3(1.0))))
        discard;

    bool solid = texture(uSolid, tc).r > 0.5;
    if (solid) {
        if (!uShowSolid)
            discard;
        FragColor = vec4(vec3(0.22), uOpacity);
        return;
    }

    vec4 flow = texture(uFlow, tc);
    float value = scalarAt(tc, flow.rgb, flow.a);
    float t = clamp((value - uRangeMin) / max(uRangeMax - uRangeMin, 1e-6), 0.0, 1.0);
    vec3 colour = texture(uColormap, t).rgb;

    if (uContours) {
        float f = t * float(uContourCount);
        float w = max(fwidth(f), 1e-6);
        float d = abs(fract(f - 0.5) - 0.5) / w;
        float line = 1.0 - smoothstep(0.0, 1.5, d);
        colour = mix(colour, vec3(0.05), line * 0.85);
    }

    FragColor = vec4(colour, uOpacity);
}
