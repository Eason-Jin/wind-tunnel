#include "render/gl/Shader.h"

#include <glm/gtc/type_ptr.hpp>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace render::gl {

std::string shaderPath(const std::string& file)
{
    return std::string(WT_SHADER_DIR) + file;
}

namespace {

std::string readFile(const std::string& file)
{
    const std::string path = shaderPath(file);
    std::ifstream in(path);
    if (!in)
        throw std::runtime_error("Cannot open shader file: " + path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

GLuint compile(GLenum stage, const std::string& file)
{
    const std::string src = readFile(file);
    const char* ptr = src.c_str();
    const GLuint s = glCreateShader(stage);
    glShaderSource(s, 1, &ptr, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        glGetShaderiv(s, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> log(static_cast<std::size_t>(len) + 1);
        glGetShaderInfoLog(s, len, nullptr, log.data());
        glDeleteShader(s);
        throw std::runtime_error("Shader compile failed (" + file + "):\n" + log.data());
    }
    return s;
}

GLuint link(std::initializer_list<GLuint> stages, const std::string& label)
{
    const GLuint p = glCreateProgram();
    for (GLuint s : stages)
        glAttachShader(p, s);
    glLinkProgram(p);
    for (GLuint s : stages) {
        glDetachShader(p, s);
        glDeleteShader(s);
    }
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        glGetProgramiv(p, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> log(static_cast<std::size_t>(len) + 1);
        glGetProgramInfoLog(p, len, nullptr, log.data());
        glDeleteProgram(p);
        throw std::runtime_error("Program link failed (" + label + "):\n" + log.data());
    }
    return p;
}

} // namespace

Shader Shader::fromFiles(const std::string& vertFile, const std::string& fragFile)
{
    const GLuint vs = compile(GL_VERTEX_SHADER, vertFile);
    GLuint fs = 0;
    try {
        fs = compile(GL_FRAGMENT_SHADER, fragFile);
    } catch (...) {
        glDeleteShader(vs);
        throw;
    }
    return Shader(link({vs, fs}, vertFile + " + " + fragFile));
}

Shader Shader::computeFromFile(const std::string& compFile)
{
    return Shader(link({compile(GL_COMPUTE_SHADER, compFile)}, compFile));
}

Shader::~Shader()
{
    if (id_)
        glDeleteProgram(id_);
}

Shader::Shader(Shader&& other) noexcept
    : id_(std::exchange(other.id_, 0)), locations_(std::move(other.locations_))
{
}

Shader& Shader::operator=(Shader&& other) noexcept
{
    if (this != &other) {
        if (id_)
            glDeleteProgram(id_);
        id_ = std::exchange(other.id_, 0);
        locations_ = std::move(other.locations_);
    }
    return *this;
}

GLint Shader::location(const char* name)
{
    auto it = locations_.find(name);
    if (it != locations_.end())
        return it->second;
    const GLint loc = glGetUniformLocation(id_, name);
    locations_.emplace(name, loc);
    return loc;
}

void Shader::set(const char* name, int v) { glProgramUniform1i(id_, location(name), v); }
void Shader::set(const char* name, float v) { glProgramUniform1f(id_, location(name), v); }
void Shader::set(const char* name, const glm::vec2& v) { glProgramUniform2fv(id_, location(name), 1, glm::value_ptr(v)); }
void Shader::set(const char* name, const glm::vec3& v) { glProgramUniform3fv(id_, location(name), 1, glm::value_ptr(v)); }
void Shader::set(const char* name, const glm::vec4& v) { glProgramUniform4fv(id_, location(name), 1, glm::value_ptr(v)); }
void Shader::set(const char* name, const glm::ivec3& v) { glProgramUniform3iv(id_, location(name), 1, glm::value_ptr(v)); }
void Shader::set(const char* name, const glm::mat3& v) { glProgramUniformMatrix3fv(id_, location(name), 1, GL_FALSE, glm::value_ptr(v)); }
void Shader::set(const char* name, const glm::mat4& v) { glProgramUniformMatrix4fv(id_, location(name), 1, GL_FALSE, glm::value_ptr(v)); }

} // namespace render::gl
