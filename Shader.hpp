#ifndef SHADER_HPP
#define SHADER_HPP

#include <GL/glew.h>
#include <string>

class Shader {
public:
    Shader();
    ~Shader();
    void Use();
    GLuint Program;

private:
    void CompileShaders();
    const std::string vertexShaderSource = R"(
        #version 330 core
        layout(location = 0) in vec3 position;
        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;
        void main() {
            gl_Position = projection * view * model * vec4(position, 1.0);
        }
    )";

    const std::string fragmentShaderSource = R"(
        #version 330 core
        out vec4 color;
        void main() {
            color = vec4(1.0, 0.0, 0.0, 1.0); // Color rojo
        }
    )";
};

#endif
