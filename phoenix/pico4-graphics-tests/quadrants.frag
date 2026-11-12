// SPDX-License-Identifier: Unlicense
#version 450
layout(location = 0) out vec4 color;
void main() {
    bool left = gl_FragCoord.x < 128;
    color = vec4(left ? 1 : 0, gl_FragCoord.y < 128 ? 0.25 : 0.75, left ? 0 : 1, 1);
}
