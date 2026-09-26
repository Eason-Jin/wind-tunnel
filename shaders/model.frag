#version 460 core
// Lit body mesh: headlight key light + hemispheric ambient + mild Blinn-Phong
// specular, two-sided (normal flipped to face the eye), gamma-corrected.
// Optionally coloured by a per-vertex scalar (surface Cp) via a 1D colormap.

in vec3 vWorldPos;
in vec3 vNormal;
in float vScalar;

out vec4 FragColor;

uniform vec3 uEye;
uniform vec3 uClayColor;
uniform int uColorMode; // 0 = clay, 1 = surface scalar (Cp)
uniform sampler1D uColormap;
uniform float uScalarMin;
uniform float uScalarMax;
uniform float uOpacity;

vec3 toSRGB(vec3 c) { return pow(clamp(c, 0.0, 1.0), vec3(1.0 / 2.2)); }

void main()
{
    vec3 n = normalize(vNormal);
    vec3 v = normalize(uEye - vWorldPos);
    // Two-sided: STL winding is unreliable, so make sure the normal faces the eye.
    if (dot(n, v) < 0.0)
        n = -n;

    vec3 baseColour = uClayColor;
    if (uColorMode == 1) {
        float t = clamp((vScalar - uScalarMin) / max(uScalarMax - uScalarMin, 1e-6), 0.0, 1.0);
        baseColour = texture(uColormap, t).rgb;
    }

    // Headlight key light: comes from the camera direction.
    vec3 l = v;
    float ndotl = max(dot(n, l), 0.0);

    // Hemispheric ambient: sky above (+z), ground below.
    vec3 skyColour = vec3(0.55, 0.60, 0.68);
    vec3 groundColour = vec3(0.20, 0.18, 0.16);
    float up = 0.5 * n.z + 0.5;
    vec3 ambient = mix(groundColour, skyColour, up);

    vec3 h = normalize(l + v);
    float ndoth = max(dot(n, h), 0.0);
    float spec = pow(ndoth, 32.0) * 0.25;

    vec3 colour = baseColour * (ambient * 0.55 + ndotl * 0.75) + vec3(spec);

    FragColor = vec4(toSRGB(colour), uOpacity);
}
