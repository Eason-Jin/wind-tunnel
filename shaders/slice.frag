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

uniform int uQuantity; // 0 = speed, 1 = Cp, 2 = Ux, 3 = vorticity magnitude, 4 = vorticity along the plane normal
uniform int uAxis;     // plane normal: 0 = x, 1 = y, 2 = z
uniform float uRangeMin;
uniform float uRangeMax;
uniform bool uShowSolid; // false => discard solid fragments
uniform bool uContours;
uniform int uContourCount;
uniform float uOpacity;

vec3 curlAt(vec3 tc)
{
    vec3 d = 1.0 / vec3(uGridDims);
    vec3 cell = uGridSize / vec3(uGridDims);
    vec3 vXp = texture(uFlow, tc + vec3(d.x, 0.0, 0.0)).rgb;
    vec3 vXm = texture(uFlow, tc - vec3(d.x, 0.0, 0.0)).rgb;
    vec3 vYp = texture(uFlow, tc + vec3(0.0, d.y, 0.0)).rgb;
    vec3 vYm = texture(uFlow, tc - vec3(0.0, d.y, 0.0)).rgb;
    vec3 vZp = texture(uFlow, tc + vec3(0.0, 0.0, d.z)).rgb;
    vec3 vZm = texture(uFlow, tc - vec3(0.0, 0.0, d.z)).rgb;
    vec3 dVdx = (vXp - vXm) / (2.0 * cell.x);
    vec3 dVdy = (vYp - vYm) / (2.0 * cell.y);
    vec3 dVdz = (vZp - vZm) / (2.0 * cell.z);
    return vec3(dVdy.z - dVdz.y, dVdz.x - dVdx.z, dVdx.y - dVdy.x);
}

// Vorticity through the plane, smoothed with a 3x3 binomial filter in the
// plane: unsteady snapshots carry a grid-scale ripple that differentiation
// amplifies, while eddies span many cells and keep most of their strength.
float spinAt(vec3 tc)
{
    int axis = clamp(uAxis, 0, 2);
    vec3 d = 1.0 / vec3(uGridDims);
    vec3 a = vec3(0.0), b = vec3(0.0);
    a[(axis + 1) % 3] = d[(axis + 1) % 3];
    b[(axis + 2) % 3] = d[(axis + 2) % 3];
    float sum = 0.0;
    for (int i = -1; i <= 1; ++i)
        for (int j = -1; j <= 1; ++j) {
            float w = (i == 0 ? 2.0 : 1.0) * (j == 0 ? 2.0 : 1.0);
            sum += w * curlAt(tc + float(i) * a + float(j) * b)[axis];
        }
    return sum / 16.0;
}

float scalarAt(vec3 tc, vec3 vel, float pressure)
{
    if (uQuantity == 4)
        return spinAt(tc);
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
