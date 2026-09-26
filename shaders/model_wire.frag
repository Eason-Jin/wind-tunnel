#version 460 core
// Flat-coloured wireframe overlay for ModelPass, paired with model.vert.
out vec4 FragColor;

uniform vec4 uWireColor;

void main()
{
    FragColor = uWireColor;
}
