#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <string>
#include <unordered_map>

namespace render::gl {

// Linked GLSL program. Files are resolved relative to the project's shaders/
// directory. Throws std::runtime_error with the compile/link log on failure.
class Shader {
public:
    Shader() = default;
    static Shader fromFiles(const std::string& vertFile, const std::string& fragFile);
    static Shader fromFiles(const std::string& vertFile, const std::string& geomFile, const std::string& fragFile);
    static Shader computeFromFile(const std::string& compFile);

    ~Shader();
    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;
    Shader(Shader&& other) noexcept;
    Shader& operator=(Shader&& other) noexcept;

    GLuint id() const { return id_; }
    explicit operator bool() const { return id_ != 0; }
    void use() const { glUseProgram(id_); }

    // Uniform setters; silently ignore uniforms the compiler optimised away.
    void set(const char* name, int v);
    void set(const char* name, float v);
    void set(const char* name, const glm::vec2& v);
    void set(const char* name, const glm::vec3& v);
    void set(const char* name, const glm::vec4& v);
    void set(const char* name, const glm::ivec3& v);
    void set(const char* name, const glm::mat3& v);
    void set(const char* name, const glm::mat4& v);

private:
    explicit Shader(GLuint id) : id_(id) {}
    GLint location(const char* name);

    GLuint id_ = 0;
    std::unordered_map<std::string, GLint> locations_;
};

// Absolute path of a file in the shaders/ directory.
std::string shaderPath(const std::string& file);

} // namespace render::gl
