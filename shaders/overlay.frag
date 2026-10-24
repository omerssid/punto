#version 460
in vec4 vColor;
out vec4 outColor;

uniform float uAlpha;

void main() { outColor = vec4(vColor.rgb, vColor.a * uAlpha); }
