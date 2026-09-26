#version 460 core
// Lit Q-criterion isosurface: same headlight + hemispheric ambient +
// mild Blinn-Phong recipe as model.frag, two-sided since marching-cubes
// winding isn't guaranteed to face any particular way. Colour is baked
// per vertex on the CPU (swirl direction or velocity magnitude).

in vec3 vWorldPos;
in vec3 vNormal;
in vec3 vColor;

out vec4 FragColor;

uniform vec3 uEye;
uniform float uOpacity;

vec3 toSRGB(vec3 c) { return pow(clamp(c, 0.0, 1.0), vec3(1.0 / 2.2)); }

void main()
{
    vec3 n = normalize(vNormal);
    vec3 v = normalize(uEye - vWorldPos);
    // Two-sided: marching-cubes winding/normal sign isn't reliable, so make
    // sure the shading normal faces the eye.
    if (dot(n, v) < 0.0)
        n = -n;

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

    vec3 colour = vColor * (ambient * 0.55 + ndotl * 0.75) + vec3(spec);

    FragColor = vec4(toSRGB(colour), uOpacity);
}
