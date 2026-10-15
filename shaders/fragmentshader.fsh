#version 330 core
in vec3 v_color;
in vec2 v_texCoord;

out vec4 FragColor;

uniform sampler2D u_texture;

void main()
{
    vec4 texColor = texture2D(u_texture, v_texCoord);
    FragColor = vec4(v_color, 1.0);
 //   FragColor = texColor * vec4(v_color, 1.0);
}
